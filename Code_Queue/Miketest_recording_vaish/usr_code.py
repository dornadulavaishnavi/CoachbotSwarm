import math
import struct

def usr(robot):
	robot.delay(3000)
	while True:
	# your looping code here
		robot.set_led(0,100,0)
		robot.set_vel(-10,10)
		robot.delay(500)
		for j in range(4):
			for i in range(5):
				robot.set_led(100,0,0)
				for i in range(5):
					robot.set_led(90,90,100)
					robot.set_vel(10,10)
					robot.delay(100)
				for i in range(5):
					robot.set_led(0,0,0)
					robot.set_vel(-10,-10)
					robot.delay(100)
			robot.set_led(0,0,100)
			robot.set_vel(-20,20)
			robot.delay(200)
		return