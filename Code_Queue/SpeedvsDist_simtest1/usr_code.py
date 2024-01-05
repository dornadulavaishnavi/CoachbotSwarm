import math
import struct

def turn_to_angle(robot, curr_theta, desired_theta):
    import math
    desired_theta_360 = (desired_theta + (2*math.pi)) % (2*math.pi)
    curr_theta_360 = (curr_theta + (2*math.pi)) % (2*math.pi)
    half_line = (curr_theta_360 + (math.pi)) % (2*math.pi)
    # robot.logger.info(half_line)
    while abs(curr_theta_360 - desired_theta_360) > 0.1:
        curr_theta_360 = (curr_theta + (2*math.pi)) % (2*math.pi)
        if curr_theta_360 > half_line:
            if (desired_theta_360 < half_line) or (desired_theta_360 > curr_theta_360):
                robot.set_vel(-30, 30)
            else:
                robot.set_vel(30, -30)
        else:
            if (desired_theta_360 < half_line) and (desired_theta_360 > curr_theta_360):
                robot.set_vel(-30, 30)
            else:
                robot.set_vel(30, -30)
        curr_pose = robot.get_pose()
        if curr_pose:
            curr_theta = curr_pose[2]
    robot.set_vel(0,0)
    return True

def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string\n")
    log.flush()

    start_pose = robot.get_pose()
    while not start_pose:
        start_pose = robot.get_pose()

    other_pose = [start_pose[0]+1, start_pose[1], start_pose[2]]
    goal_pose = other_pose
    starting_vel = 10
    loop_num = 1
    switch_pose_flag = 0
    start_time = robot.get_clock()

    while True:
        robot.delay()
        # Keep moving until at final position
        curr_pose = robot.get_pose()
        while not curr_pose:
            curr_pose = robot.get_pose()
        if curr_pose[0] - goal_pose[0] < 0.1:
            switch_pose_flag = 1
        else:
            robot.set_vel(starting_vel*loop_num, starting_vel*loop_num)
        
        # if at final position, turn and set new position, increment loop count
        if switch_pose_flag == 1:
            log.write(str(robot.get_clock-start_time))
            log.flush()
            robot.set_vel(0,0)
            if loop_num%2 == 0:
                goal_pose = start_pose
            turn_to_angle(robot, curr_pose[2], (curr_pose[2]+(math.pi))%(2*math.pi))
            loop_num += 1
            switch_pose_flag = 1
            start_time = robot.get_clock()

        # check if we're done trying full range of wheel speeds
        if loop_num >= 5:
            log.close()
            return
        