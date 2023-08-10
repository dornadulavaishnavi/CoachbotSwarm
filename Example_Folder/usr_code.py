import math
import struct

def usr(robot):
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string")
    log.flush()

    while True:
        robot.delay()
        # your looping code here

        log.close()
        return