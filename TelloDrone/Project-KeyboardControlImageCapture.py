from djitellopy2 import tello
import KeyPressModule as kp
import time
import cv2
from time import sleep

kp.init()
drone = tello.Tello()
drone.connect()
print(drone.get_battery())
global img
drone.streamon()
def getKeyBoardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 100

    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = -speed
    elif kp.getKey("d"): yv = speed

    if kp.getKey("q"): drone.land(); time.sleep(3)
    if kp.getKey("e"): drone.takeoff()

    if kp.getKey("z"):
        cv2.imwrite(f'Resources/Images/{time.time()}.jpg',img)
        time.sleep(3)

    return[lr, fb, ud, yv]


while True:
    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    img = drone.get_frame_read().frame
    img = cv2.resize(img, (360,240))
    cv2.imshow("Image",img)
    cv2.waitKey(1)

