# ROS-Tello-tracker
基於 ROS (Robot Operating System) 的無人機視覺追蹤任務  
此專案結合了 AprilTag 偵測與紅色方框追蹤技術，能夠協助無人機自主定位與導航。  

## 功能特色
- 即時接收無人機相機的 H264 壓縮影像串流  
- 使用 AprilTag 演算法進行標籤偵測  
- 使用 HSV 色彩空間進行紅色方框追蹤  
- 發布目標中心點座標與大小比例給 ROS topic，方便其他節點使用  
- 支持 OpenCV 影像視窗即時顯示追蹤結果  
- 可記錄追蹤影片以便後續分析  

## 環境需求
- Ubuntu 18.04 或以上  
- ROS (建議 melodic 或 noetic)  
- Python 3.x  
- OpenCV  
- PyAV  
- AprilTag (Python binding)  

