#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # 導入套件: rospy
from sensor_msgs.msg import CompressedImage   # 導入 sensor_msgs 裡的 CompressedImage
from std_msgs.msg import Float64MultiArray    # 導入 std_msgs 裡的 Float64MultiArray
import av                                     # 導入套件: av
import cv2                                    # 導入套件: cv2
import math                                   # 導入套件: math
import numpy as np                            # 導入套件: numpy 並命名成 np
import threading                              # 導入套件: threading
import traceback                              # 導入套件: traceback
import time                                   # 導入套件: time
import apriltag 

TARGET_RATIO = 0.35 # 定義無人機可以加速穿越方框的比例門檻
TARGET_SIZE = 160 # 定義無人機在目標位置時的apriltag尺寸

### class StandaloneVideoStream 
### 用於將訂閱/tello/image_raw/h264所獲得的 CompressedImage 進行處理, 得到stream
### 再從stream中取得影像
class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()

def detect_frame(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 定義 HSV 值域的紅色上下界(需要定義兩次)
    lr0, ur0 = np.array([0, 70, 0]),  np.array([5, 255, 255]) # 定義 HSV 下界 & 上界
    lr1, ur1 = np.array([175, 70, 0]), np.array([180, 255, 255]) # 定義 HSV 下界 & 上界
    mask0, mask1 = cv2.inRange(hsv_img, lr0, ur0), cv2.inRange(hsv_img, lr1, ur1) # 建立遮罩1, 遮罩2

    mask = cv2.bitwise_or(mask0, mask1) # 將兩組遮罩進行or運算合併

    # 對找出的HSV mask 進行輪廓搜尋: findContours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # max function 在遇上空陣列[]會報錯, 導致程式終止, 
    # 如要避免此情況可以在執行max()前透過continue來避免 
    if len(contours) == 0:
        print("not found")
        return None, None, None, None
 
    max_contour = max(contours, key = cv2.contourArea)

    # 使用boundingRect()找出可以包覆max_contour的最小矩形(平行軸)
    x, y, w, h = cv2.boundingRect(max_contour) # 取得座標與長寬尺寸

    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2) # 將包覆矩形繪製在影像上
    cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0,0,255), -1) # 繪製方形的中心點
    cv2.circle(image, (480, 200), 5, (0,255,0), -1) # 繪製鏡頭的中心點
    return x+w/2, y+h/2, w*h

def detect_apriltag(image):
    # 建立 aprilTag Detector
    detector = apriltag.Detector()
    # detector接收灰階圖像進行偵測, 這裡將轉換成gray格式
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 使用 aprilTag Detector 進行 aprilTag偵測
    result = detector.detect(gray_image)
        
    # 複製一份 原始影像image 作為 show_image
    show_image = image.copy()
    # 當 沒有偵測到 aprilTag, 在視窗上顯示 "not found"
    # 當 有偵測到 aprilTag, 取出其中心點 center 座標
    if len(result) == 0:
        print("not found")
        return  0, 0, 0
    else:
          # result 為 1 list, 故透過 [0] 取出第一個 aprilTag
          center = result[0].center
          corner = result[0].corners
         # 透過左下角與右下角計算寬 w
          w = math.sqrt( (corner[0][0] - corner[1][0])**2 + (corner[0][1] - corner[1][1])**2 )
          
          # 透過右下角與右上角計算寬 h
          h = math.sqrt( (corner[2][0] - corner[1][0])**2 + (corner[2][1] - corner[1][1])**2 )
   
    cv2.circle(image, (int(center[0]),int(center[1])) , 5, (0,0,255), -1)
    cv2.polylines(image,[np.int32(corner)], True, (0,0,255), 2, cv2.LINE_AA) 	      
    cv2.circle(image, (480, 200), 5, (0,255,0), -1) # 繪製鏡頭的中心點
    return center[0], center[1], w*h

def main():
    # fourcc: video的編碼格式, 如 XVID, MP4V 等等...
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    # out: 建立 VideoWriter, video名稱為 test.avi, 寫入格式為 'X',"V",'I','D', FPS 為 20.0, video解析度為 (影像寬, 影像高)
    out = cv2.VideoWriter('follow_demo.avi', fourcc, 20.0, (960, 720))

   

    # 告訴ros此程式為node, node名稱為 'h264_listener'
    rospy.init_node('h264_listener')

    stream = StandaloneVideoStream() # 建立 stream

    # 定義 rospy.Subscriber, 會訂閱 topic: '/tello/image_raw/h264'
    # CompressedImage為 topic: '/tello/image_raw/h264' 所需要的訊息格式, 也就是無人機獲取的影像
    # compessed_image_callback 為接收訊息與處理function的名字, 可以自行定義, 但需一致
    # rospy.Subscriber("/tello/image_raw/h264", CompressedImage, compessed_image_callback)
    # 因為 callback function 內容只有一行statement, 這裡用 lambda 來建立
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, lambda msg: stream.add_frame(msg.data))

    # 建立rospy.Publisher 名稱'point_pub', 將 Float64MultiArray 訊息發布至 topic: /target_point 上.
    # queue_size表示暫存訊息的queue大小，
    # 當發布的訊息量超過queue_size時，就會開始剔除舊訊息
    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)

    # 建立變數 ap_pub, 建立rospy.Publisher, 將 Float64MultiArray 訊息發布至 topic: /target_ap 上. 
    # queue_size表示暫存訊息的queue大小，
    # 當發布的訊息量超過queue_size時，就會開始剔除舊訊息
    ap_pub = rospy.Publisher("/target_ap", Float64MultiArray, queue_size = 10)
    

    container = av.open(stream) # 用PyAV的open()建立容器接收視訊串流

    # 由於 stream 會接收一開始執行時就接收到的 compressedImg, 
    # 因此會有一些過時的 compressedImg 需要跳過, 先設定跳過 300 幀
    frame_skip = 300
    current_task = 0 # 定義當前的任務: 0 代表偵測方框, 1 代表偵測apriltag
    start_detect = True # 建立 bool 變數 start_detect

    # 在視窗上顯示 log: 'main: opened'
    rospy.loginfo('main: opened')
    
    # 對container的每一幀進行解碼, 獲取已被加入至stream中的影像
    # 當container解碼後發現沒有影像將會終止迴圈, 也代表 此程式會結束執行
    for frame in container.decode(video=0):
    
        # 當 frame_skip > 0, 則透過 continue 跳過該次影像處理迴圈, 並 frame_skip -1 
        if frame_skip > 0:
            frame_skip -= 1
            continue
        
        # 計算影像處理迴圈一次所需要花費的時間, 先建立 start_time 作為起始
        start_time = time.time()
        
        # 透過np.array將frame先轉成np格式, 再將numpy的RBG格式改成opencv的BGR格式
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
    
        print("current_task:", current_task)

        # 當 start_detect 為 True
        if start_detect:
            if current_task == 0: # 當前偵測對象為方框
                frame_data = detect_frame(image) 
                # 確保 x 坐標不是 None , 代表已經成功偵測到方框
                if frame_data[0] is not None:
                    # (w * h)/(960*720) 為方形面積/總畫面面積, 兩者相除後得到 方形面積在總畫面中的占比, 透過str()將float轉成 string
                    ratio = frame_data[2] / (960*720.0)
                    # 當 mask面積 / 畫面總面積 (960 * 720) 大於等於 門檻值 0.35, 表示無人機已經夠接近框, 可以進行加速過框行為
                    if ratio >= TARGET_RATIO:
                        # 透過 point_pub 發布 Float64MultiArray訊息, 將 框的 中心點x, 中心點y, 1發佈出去, 1表示占比已超過門檻值
                        point_pub.publish(Float64MultiArray(data = [frame_data[0], frame_data[1], 1]))
                        print("Start fly through the frame")
                        # 可加速通過框後就不再發佈訊息, 將 start_detect 改為 False
                        #start_detect = False
                        current_task = 1

                    # 無人機還不夠接近框, 可以繼續進行校正與向前
                    else:
                        # 透過 point_pub 發布 Float64MultiArray訊息, 將 框的 中心點x, 中心點y, 0發佈出去, 0表示占比未超過門檻值
                        point_pub.publish(Float64MultiArray(data = [frame_data[0], frame_data[1], 0])) 
                        
            elif current_task == 1: # 當前偵測對象為 apriltag 
                apriltag_data = detect_apriltag(image) # 得到 x, y, w, h	        
                print(apriltag_data[2])
                
                # 表示無人機已經位於apriltag前正確距離, 可以進行下降過框行為
                if apriltag_data[2] >= 20000:
                        # 透過 point_pub 發布 Float64MultiArray訊息, 將apriltag的 中心點x, 中心點y, 1(表示占比已超過門檻值)發佈出去
                        ap_pub.publish(Float64MultiArray(data = [apriltag_data[0], apriltag_data[1], 1]))
                        print("Start Turn left or Go down")
                        # 可加速通過框後就不再發佈訊息, 將 start_detect 改為 False
                        #start_detect = False

                # 無人機還不夠接近框, 可以繼續進行校正與向前
                else:
                        # 透過 point_pub 發布 Float64MultiArray訊息, 將apriltag的  中心點x, 中心點y, 0(表示占比未超過門檻值)發佈出去
                        ap_pub.publish(Float64MultiArray(data = [apriltag_data[0], apriltag_data[1], 0]))
                       
                       
            

        # 將影像寫入到video: out中
        out.write(image)    
        
        cv2.imshow('test_window', image)
        
        # 設定視窗刷新頻率
        key = cv2.waitKey(1)
        if key == ord('s'):
            start_detect = True
            print("Start detect!")
        if key == ord('q'):
            break

        # 計算 stream中的FPS
        if frame.time_base < 1.0/60:
            time_base = 1.0/60
        else:
            time_base = frame.time_base

        frame_skip = int((time.time() - start_time)/time_base)
    stream.close()

# main
if __name__ == '__main__':

    # try catch
    try:
        # 執行 main fuction, 啟動影像處理迴圈
        main()
    # 例外處理: BaseException, 有發生會透過 traceback 顯示例外錯誤
    except BaseException:
        traceback.print_exc()
    
    # 最終處理: 關掉 stream, 並將cv2所產生的影像視窗都關掉
    finally:
        cv2.destroyAllWindows()
