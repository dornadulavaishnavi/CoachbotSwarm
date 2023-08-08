import math
import struct

def usr(robot):

	while True:
	# your looping code here
		for i in range(5):
			robot.set_led(100,0,100)
			robot.set_vel(-20,20)
			robot.delay(5000)
			robot.set_led(0,0,0)
			robot.set_vel(0,0)
			robot.delay(5000)

	return