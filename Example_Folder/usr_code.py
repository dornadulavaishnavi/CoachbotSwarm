import math
import struct

def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string\n")
    log.flush()

    while True:
        robot.delay()
        # your looping code here

        log.close()
        return