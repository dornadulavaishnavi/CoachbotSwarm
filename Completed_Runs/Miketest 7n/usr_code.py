import math
import struct

def usr(robot):

	while True:
	# your looping code here
		robot.set_led(0,100,0)
		robot.set_vel(-10,10)
		robot.delay(500)
		for i in range(5):
			for i in range(5):
				robot.set_led(90,90,100)
				robot.set_vel(10,10)
				robot.delay(100)
			for i in range(5):
				robot.set_led(0,0,0)
				robot.set_vel(-10,-10)
				robot.delay(100)

		return