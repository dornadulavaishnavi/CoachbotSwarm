import math
import struct

def usr(robot):

	while True:
	# your looping code here
		for i in range(5):
			for i in range(5):
				robot.set_led(100,0,100)
				robot.set_vel(-20,20)
				robot.delay(1100)
			for i in range(5):
				robot.set_led(0,0,0)
				robot.set_vel(15,-15)
				robot.delay(1100)

		return