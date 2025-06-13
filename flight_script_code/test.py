from time import sleep
from djitellopy import Tello


def fly_poly(drone):
    for i in range(2):
        #(左右,前後,上下,旋轉) -100~100
        drone.send_rc_control(0, 50, 0, 0)
        sleep(2)
        drone.send_rc_control(0, 0, 30, 0)
        sleep(2)
        drone.send_rc_control(0, 0, 0, 0)
        sleep(1)
        drone.send_rc_control(0, 0, 0, 60)
        sleep(2)
        drone.send_rc_control(0, 0, 0, 0)
        sleep(1)
        drone.send_rc_control(0, 40, 0, 0)
        sleep(2)
        drone.send_rc_control(0, 0, 0, 0)
        sleep(1)
        drone.send_rc_control(0, 0, 0, 60)
        sleep(2)
        drone.send_rc_control(0, 0, 0, 0)
        sleep(1)

def fly_infinite(drone):
    #20~500cm

    drone.move_up(50)
    sleep(1)
    print("300cm foward")
    drone.move_forward(300)  
    sleep(2)
    drone.move_down(20)
    sleep(1)
    drone.flip_forward()
    sleep(1)
    print("向右旋轉135度")
    drone.rotate_clockwise(135)
    sleep(1)

    print("250cm foward")
    drone.move_forward(250)
    sleep(2)
    drone.flip_right()  
    sleep(1)
    drone.move_left(100)
    sleep(1)
    drone.move_up(20)
    sleep(1)
    print("向左旋轉135度")
    drone.rotate_counter_clockwise(135)
    sleep(1)

    print("200cm foward")
    drone.move_forward(200)
    sleep(2)
    drone.flip_left() 
    sleep(1) 
    drone.move_right(100)4444444444444444444444444444444444444444444445
    sleep(1)
    print("向右旋轉45度")
    drone.rotate_clockwise(45)
    sleep(1)

    print("250cm backward")
    drone.move_back(250)
    sleep(2)
    drone.move_down(20)
    sleep(1)
    drone.flip_back()   
    drone.rotate_clockwise(180)
    drone.move_forward(100)
    




def test():      
    drone = Tello()         
    drone.connect()  
    
    try:
        print(f"battery: {drone.get_battery()}%") 
        print(drone.get_current_state())
        drone.takeoff()
        fly_infinite(drone)
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()

    except Exception as ex:
        print(ex)

    finally:
        drone.end()


if __name__ == '__main__':
    test()
    
    