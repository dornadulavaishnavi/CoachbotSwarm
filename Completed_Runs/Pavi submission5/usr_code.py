#user code file

import math
import struct

def usr(robot):
  robot.delay(3000) 

  while True:
    robot.delay()
    # your looping code here
    for i in range(5):

      robot.set_led(100,100,100)
      robot.set_vel(-5,5)
      robot.delay(2)
      robot.set_vel(0,0)
      robot.set_led(0,0,0)
      robot.set_vel(5,-5)
      robot.delay(2)
      robot.set_vel(0,0)
      robot.delay(2)
      robot.set_vel(5,5)
      robot.delay(2)
      robot.set_vel(0,0)
    return