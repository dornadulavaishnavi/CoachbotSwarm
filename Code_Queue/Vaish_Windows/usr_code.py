import math
import struct
import numpy as np
import sys

robot_radius = 0.055 #0.07 #0.055
D = robot_radius*0.5
L = (robot_radius*2)-0.04

P_gain = 0
D_gain = 0
I_gain = 0
I_windup_max = 15

def usr(robot):
    desired_vels = {1:[-0.1,0.1], 2:[2,4]}
    u = [0,0]
    integrated_error_x = 0
    integrated_error_y = 0
    actual_prev_vel_x = 0
    actual_prev_vel_y = 0

    while True:
        robot.delay()
        curr_pose = robot.get_pose()
        while not curr_pose:
            curr_pose = robot.get_pose()
            
        vive_loc_first = get_indv_vive_loc(curr_pose)
        first_position_timestamp = robot.get_clock()
        
        robot.relay(500)

        theta = curr_theta
        M = np.array([[((math.cos(theta)/2)+(D*math.sin(theta)/L)), ((math.cos(theta)/2)-(D*math.sin(theta)/L))],[((math.sin(theta)/2)-(D*math.cos(theta)/L)), ((math.sin(theta)/2)+(D*math.cos(theta)/L))]])
        M_inv = np.linalg.inv(M)

        curr_pose = robot.get_pose()
        while not curr_pose:
            curr_pose = robot.get_pose()

        vive_loc_second = get_indv_vive_loc(curr_pose)
        second_position_timestamp = robot.get_clock()

        vive_update_rate = (second_position_timestamp-first_position_timestamp)
        # left wheel
        actual_vel_x = (vive_loc_second[0][0]-vive_loc_first[0][0])/vive_update_rate
        actual_vel_y = (vive_loc_second[0][1]-vive_loc_first[0][1])/vive_update_rate
        v_actual = np.array([actual_vel_x, actual_vel_y])

        error_x = desired_vel[0][0] - actual_vel_x
        error_y = desired_vel[0][1] - actual_vel_y

        vel_x = (actual_vel_x-actual_prev_vel_x)/vive_update_rate
        vel_y = (actual_vel_y-actual_prev_vel_y)/vive_update_rate

        integrated_error_x = integrated_error_x + (error_x*I_gain)
        integrated_error_y = integrated_error_y + (error_y*I_gain)
        robot.logger.info()

        x_vel = desired_vel[0][0] + (error_x*P_gain) + (vel_x*D_gain) + (integrated_error_x)
        y_vel = desired_vel[0][1] + (error_y*P_gain) + (vel_y*D_gain) + (integrated_error_y)

        # u = M_inv.dot([x_vel,y_vel])
        # robot.set_vel(u[0],u[1])
        # robot.logger.info(str(robot.id) + " : new wheel speeds " + str(vl) + " , " + str(vr))

        actual_prev_vel_x = actual_vel_x
        actual_prev_vel_y = actual_vel_y

def get_indv_vive_loc(curr_pose):
    vive_loc = [[curr_pose[0]-wheel_dist*math.sin(curr_pose[2]),curr_pose[1]+wheel_dist*math.cos(curr_pose[2])],[curr_pose[0]+wheel_dist*math.sin(curr_pose[2]),curr_pose[1]-wheel_dist*math.cos(curr_pose[2])]]
    return vive_loc