import math

from djitellopy import tello
import KeyPressModule as kp
import numpy as np
import cv2
import math
from time import sleep

#PARAMETERS#
fspeed = 117/10 #approx forward speed in cm/s
anguSpeed = 360/10 # apprx angular speed Deg/s
interval = 0.25

dInterval = fspeed*interval
aInterval = anguSpeed*interval
#############
x,y = 500, 500
angle = 0
yaw = 0

kp.init()
drone = tello.Tello()
drone.connect()
print(drone.get_battery())

points = [(0,0), (0,0)]


def getKeyBoardInput():
    lr, fb, ud, yv = 0, 0, 0, 0

    speed = 10
    aspeed = 50
    distance = 0
    global x, y, yaw, angle

    if kp.getKey("LEFT"):
        lr = -speed
        distance = dInterval
        angle = -180
    elif kp.getKey("RIGHT"):
        lr = speed
        distance = -dInterval
        angle = 180

    if kp.getKey("UP"):
        fb = speed
        distance = dInterval
        angle = 270
    elif kp.getKey("DOWN"):
        fb = -speed
        distance = -dInterval
        angle = -90


    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"):
        yv = -aspeed
        yaw -= aInterval

    elif kp.getKey("d"):
        yv = aspeed
        yaw += aInterval

    if kp.getKey("q"): drone.land()
    if kp.getKey("e"): drone.takeoff()

    sleep(interval)
    angle += yaw
    x += int(distance * math.cos(math.radians(angle)))
    y += int(distance * math.sin(math.radians(angle)))

    return [lr, fb, ud, yv, x, y]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0,0,255), cv2.FILLED )
    cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {(points[-1][1]-500)/100})m',
                (points[-1][0]+10, points[-1][0]+10), cv2.FONT_HERSHEY_PLAIN, 1,
                (255,0,255), 1)
while True:
    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    img = np.zeros((1000, 1000, 3), np.uint8)
    if (points[-1],[0] != vals[4]or points [-1][1]!=vals[5]):
        points.append((vals[4],vals[5]))
    points.append((vals[4], vals[5]))
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
