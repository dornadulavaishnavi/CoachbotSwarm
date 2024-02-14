#user code file

import math
import struct

def usr(robot):
  robot.delay(3000) 

  while True:
    robot.delay()
    # your looping code here
    for i in range(5):
      robot.set_vel(0.1,0.1)
      robot.delay(1)
      robot.set_vel(0,0)
      robot.delay(1)
      robot.set_vel(-0.1,-0.1)
      robot.delay(1)
      robot.set_vel(0,0)
      robot.delay(1)
    
    return