import math
import struct

def usr(robot):
    # any set up variables or code before looping can go here
    while True:
        robot.delay()
        # your looping code here
        for i in range(10):
            robot.set_led(100,0,100)
            robot.delay(2000)
        return