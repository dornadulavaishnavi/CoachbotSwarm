import random
import math
import struct

# Current setup: fixed amount of food, robots all move around until all the food is collected

def usr(robot): 
    robot.log("*****NEW TRIAL*****")
    id = robot.id

    if id == 0:
        robot.set_led(0, 0, 100) # Blue
        food_list = []
    else:
        robot.set_led(0, 100, 0) # Green
    
    start_time = robot.get_clock()

    spawn_rate = 50

    while True:
        if id == 0: # This is the food robot, it broadcasts food item positions every <spawn_rate> seconds
            # Step 1: Generate a random food position every _ seconds
            curr_time = robot.get_clock()
            if curr_time - start_time >= spawn_rate:
                food_x = random.uniform(-4.9, 4.9) 
                food_y = random.uniform(-4.9, 4.9) 
                food_list.append((food_x, food_y, curr_time))
                start_time = curr_time

            # Step 2: Receive positions of robots, check if any of them have collected food
            msgs = robot.recv_msg()
            for msg in msgs:
                recvd_pose = struct.unpack('ff', msg)

                for food_posn in food_list:
                    dist_to_bot = (recvd_pose[0] - food_posn[0])**2 + (recvd_pose[1] - food_posn[1])**2 # Omit square root <= not necessary for comparison
                    if dist_to_bot < 0.02: 
                        time_of_collection = robot.get_clock() # This is the time at which the food was collected
                        food_lifespan = time_of_collection - food_posn[2] # This is the duration of time for which the food item has existed 

                        robot.log([time_of_collection, food_lifespan])
                        food_list.remove(food_posn)
        else:
            # Move randomly
            my_pose = robot.get_pose()
            if my_pose:
                # Generate a random motion vector
                # Generate a random angle between 0 and pi radians or 0 and -pi radians
                angle = random.uniform(0, 1) * math.pi * 2 - math.pi
                # angle = random.uniform(0, 360) # Generate random angle (0 to 360 degrees)
                # angle = math.radians(angle)

                magnitude = random.uniform(0, 100) # Genereate random magnitude (0 to 100)

                # Spin quickly until theta is at the angle
                while True:
                    my_pose = robot.get_pose()

                    # Check if my_pose is not False and the current angle is not equal to the desired angle
                    if my_pose:
                        if angle - 0.15 <= my_pose[2] <= angle + 0.15:
                            break
                        else:
                            robot.set_vel(100, -100)  # Spin in place
                
                # Move forward by "magnitude" velocity for 2 seconds
                v_start_time = robot.get_clock()
                curr_time = v_start_time
                while curr_time - v_start_time < 2:
                    robot.set_vel(magnitude, magnitude)
                    curr_time = robot.get_clock()
                
            # Broadcast position
            my_pose = robot.get_pose()
            if my_pose:
                robot.send_msg(struct.pack('ff', my_pose[0], my_pose[1]))