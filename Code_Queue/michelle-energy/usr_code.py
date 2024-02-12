import random
import time
import struct
import math
from datetime import datetime, timedelta
import os

# Current setup: fixed amount of food, robots all move around until all the food is collected

def usr(robot): 
    robot.log("*****NEW EXPERIMENT*****", "time_log.txt")
    robot.log("*****NEW EXPERIMENT*****", "energy_log.txt")
    id = robot.id

    if id == 0:
        robot.set_led(0, 0, 100) # Blue
        food_list = []
    else:
        robot.set_led(0, 100, 0)
    
    start_time = robot.get_clock()
    global_start_time = start_time

    while True:
        if id == 0: # This is the food robot, it broadcasts stuff
            # Step 1: Generate a random food position every _ seconds
            curr_time = robot.get_clock()
            if curr_time - start_time >= 1:
                food_x = random.uniform(-4, 4) 
                food_y = random.uniform(-4, 4) 
                food_list.append((food_x, food_y, curr_time))
                start_time = curr_time

            # Step 2: Receive positions of robots, check if any of them have collected food
            msgs = robot.recv_msg()
            for msg in msgs:
                recvd_pose = struct.unpack('ff', msg)

                for food_posn in food_list:
                    dist_to_bot = math.sqrt((recvd_pose[0] - food_posn[0])**2 + (recvd_pose[1] - food_posn[1])**2)
                    if dist_to_bot < 0.25: 
                        # Collide! NOTE: For now, j hard code in the value => would be nice if we could have radius field in Coachbot?
                        time_of_collection = robot.get_clock()
                        collection_time = time_of_collection - food_posn[2] # Log time

                        robot.log(f"{collection_time}", "energy_log.txt")
                        robot.log(f"{time_of_collection - global_start_time}", "time_log.txt") # NEW: For sim time testing
                        food_list.remove(food_posn)
        else:
            # Move randomly
            rv, lv = random.randint(-10, 100), random.randint(-10, 100)
            robot.set_vel(rv, lv)

            # Broadcast position
            my_pose = robot.get_pose()
            robot.send_msg(struct.pack('ff', my_pose[0], my_pose[1]))