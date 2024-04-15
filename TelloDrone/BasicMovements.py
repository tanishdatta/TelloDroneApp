from djitellopy import tello
from time import sleep

drone = tello.Tello()
drone.connect()

print(drone.get_battery())
drone.streamoff()
drone.takeoff()
sleep(3)
drone.move("forward", 100)
drone.move("up", 30)
drone.rotate_clockwise(90)
drone.flip_right()
drone.streamoff()

sleep(2)
drone.land()