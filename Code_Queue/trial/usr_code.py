import math
import struct
import numpy as np
import sys

time_step = 2.0 # 0.05 # 0.02
velocity_time_step = 1.2 # 0.95
message_time = 0.05 # 0.05
robot_radius = 0.05 #0.045 #0.055
final_tolerance = 0.01
D = robot_radius*0.35 #0.25
L = (robot_radius*2) - 0.04
effective_robot_radius = robot_radius+D
message_radius = robot_radius*4 #8
responsibility = 0.3
grid_resolution = 0.1
grid_bound = 4 # 6
expanded_grid_resolution = 0.15
expanded_grid_bound = 15

P_gain = 15
P_gain_speed = 0.1 #450 #350
D_gain = 0
I_gain = 0.01
I_windup_max = 10

max_wheel_speed = 50
min_wheel_speed = 10
square = 0

def usr(robot):
    robot.logger.info("started program")    
    incoming_dict = {}
    boundary = {}
    dict_ORCA = {}
    mid_vel = 1
    vl = mid_vel
    vr = mid_vel
    invalid = 0
    desired_theta = 0
    desired_point_flag = 1
    waypoint_index = 0
    waypoint_array = [[-0.3,-0.2,0],[0.3,-0.2,0],[-0.3,0.2,0],[0.3,0.2,0]]
    if square == 1:
        num_angle_steps = 200
        desired_angle_array = [0, -math.pi/2, 0, math.pi/2]
        angle_step_count = 0
    else:
        num_angle_steps = 200
        desired_angle_array = []
        for i in range(2):
            for j in range(num_angle_steps):
                if i == 0:
                    desired_angle_array.append(0+(((math.pi/8)/num_angle_steps)*j))
                # elif i == 1:
                #     desired_angle_array.append(-math.pi/2)
    angle_index = 0
    robot.set_led(0,0,255)
    first_flag = 1
    integrated_error = [0,0,0,0]
    speed_filter = [0,0]
    within_other_robot_dist_flag = 0

    curr_pose = robot.get_pose()
    while not curr_pose:
        curr_pose = robot.get_pose()
    position_filter_prev = np.array([curr_pose[0],curr_pose[1],curr_pose[0],curr_pose[1]])
    
    while True:
        robot.delay()
        if square == 1:
            if angle_step_count > num_angle_steps:
                angle_step_count = 0
                angle_index = (angle_index+1)%(len(desired_angle_array))
            
        curr_pose = robot.get_pose()
        while not curr_pose:
            curr_pose = robot.get_pose()
        curr_x = curr_pose[0]
        curr_y = curr_pose[1]
        curr_theta = curr_pose[2]  
        prev_pose = curr_pose
        first_position_timestamp = robot.get_clock()
        robot.logger.info("Current Position: " + str(curr_pose))

        X = (curr_x + D*math.cos(curr_theta))
        Y = (curr_y + D*math.sin(curr_theta))
        robot.logger.info("Current pose: " + str(curr_pose) + "," + str(X) + "," + str(Y))
        # robot.logger.info(str(desired_theta) + "," + str(curr_theta) + "," + str(X) + "," + str(Y))
        # robot.logger.info("\n\ncurrent position: " + str(curr_pose))

        # u = [vl,vr]
        # M = np.array([[((math.cos(desired_theta)/2)+(D*math.sin(desired_theta)/L)), ((math.sin(desired_theta)/2)-(D*math.cos(desired_theta)/L))],[((math.cos(desired_theta)/2)-(D*math.sin(desired_theta)/L)), ((math.sin(desired_theta)/2)+(D*math.cos(desired_theta)/L))]])
        # v = M.dot(u) # [x_dot,y_dot]

        # desired_pose = waypoint_array[waypoint_index]
        desired_pose = N_init_positions_skinny(robot, robot.id)
        # robot.logger.info("desired position: " + str(desired_pose))
        if desired_pose == False:
            desired_x = curr_x
            desired_y = curr_y
            final_theta = math.pi
        else:
            desired_x = desired_pose[0]
            desired_y = desired_pose[1]
            final_theta = desired_pose[2]
        # # robot.logger.info("done with desired position")

        own_opt_x = desired_x-X
        own_opt_y = desired_y-Y
        dist = math.sqrt((X-desired_x)**2 + (Y-desired_y)**2)
         
        # own_opt_x = own_opt_x/(math.sqrt(own_opt_x**2+own_opt_y**2))
        # own_opt_y = own_opt_y/(math.sqrt(own_opt_x**2+own_opt_y**2))
        desired_theta = math.atan2(own_opt_y, own_opt_x)
        # [own_opt_x,own_opt_y, desired_theta] = compute_own_velocity([curr_x,curr_y], [desired_x,desired_y])
        robot.logger.info(str(robot.id) + " : " + str(desired_theta * (180 / math.pi)) + " own optimal angle")

        own_opt_x = math.cos(desired_theta)
        own_opt_y = math.sin(desired_theta)
        
        x_dot = own_opt_x/velocity_time_step
        y_dot = own_opt_y/velocity_time_step
        theta_dot = (desired_theta - curr_theta)/velocity_time_step
        robot.logger.info("Optimal Velocities- [" + str(x_dot) + ", " + str(y_dot) + "] theta: " + str(theta_dot))
        # v = [own_opt_x,own_opt_y]

        # send robot location
        # robot.send_msg(struct.pack('ifffff', robot.id, curr_x, curr_y, curr_theta, own_opt_x, own_opt_y))

        # robot.logger.info("own location message send, starting receiving")

        dist = math.sqrt((X-desired_x)**2 + (Y-desired_y)**2)
        # while dist > final_tolerance:
        # get messages of other robot locations
        last = robot.get_clock()
       
        while(robot.get_clock()-last) < message_time:
            for i in range(2):             
                # send message with pose
                # to_send = package_string([robot.id, curr_x, curr_y, curr_theta, own_vopt_x, own_vopt_y])
                # did_send = robot.send_msg(to_send)
                # # robot.logger.info(did_send)
                # robot.delay()
                if first_flag == 1:
                    robot.send_msg(struct.pack('ifffff', robot.id, curr_x, curr_y, curr_theta, x_dot, y_dot))
                else:
                    robot.send_msg(struct.pack('ifffff', robot.id, curr_x, curr_y, curr_theta, (effective_x_dot), (effective_y_dot)))
                robot.delay()
            first_flag = 0

            rec_str = robot.recv_msg()
            robot.delay()
            if len(rec_str) > 0:
                check_string = unpack_other_pos_string(robot, rec_str)
                if len(check_string) > 2:                            
                        passive_position = check_string
                        # robot.logger.info(len(passive_position)/6)
                        i = 0
                        while i <(len(passive_position)-5):
                            if abs(passive_position[i]) < 99:    
                                incoming_dict[passive_position[i]] = [passive_position[i+1], passive_position[i+2], passive_position[i+3], passive_position[i+4], passive_position[i+5]]
                            i = i+6
                        # robot.logger.info("sending")
        
        # robot.logger.info("Done receiving messages")
        robot.logger.info(incoming_dict)
        
        updated_x = x_dot
        updated_y = y_dot
        boundary = {}
        dict_ORCA = {}
        for bot in incoming_dict:
            dist_check = math.sqrt((X-incoming_dict[bot][0])**2 + (Y-incoming_dict[bot][1])**2)
            robot.logger.info(bot)  
            if dist_check < message_radius:
                within_other_robot_dist_flag = 1
                # construct ORCA AiAj
                # compute velocity obstacle cone
                boundary[bot] = compute_Vo(robot, [X,Y], [incoming_dict[bot][0], incoming_dict[bot][1]])
                # robot.logger.info("computed velocity obstacle")
                # robot.logger.info(boundary[bot])
                # find closest point on boundary from va-vb
                w = compute_w(robot, boundary[bot], [updated_x, updated_y], [incoming_dict[bot][3],
                                            incoming_dict[bot][4]])
                robot.logger.info("computing w: " + str(w))
                # compute outward normal of boundary at point va-vb+u
                n = compute_n(boundary[bot], w, updated_x, updated_y, incoming_dict[bot][3],
                                            incoming_dict[bot][4])
                robot.logger.info("computing n: " + str(n))
                # compute orca
                dict_ORCA[bot] = gen_ORCA_line(robot, boundary[bot], n, w, updated_x, updated_y)
                robot.logger.info("ORCA line generated: " + str(dict_ORCA[bot]))
        try:
            robot.logger.info("VO: " + str(boundary))
            robot.logger.info("orca dict: " + str(dict_ORCA))
        except:
            pass
        
        # if within_other_robot_dist_flag == 1:
        invalid = 0
        grid_array = generate_grid(grid_resolution,grid_bound,grid_bound)
        orca_grid_array = update_grid(grid_array, dict_ORCA, boundary)
        if len(orca_grid_array) == 0:
            expanded_grid_array = generate_grid(expanded_grid_resolution,expanded_grid_bound,expanded_grid_bound)
            expanded_orca_grid_array = update_grid(expanded_grid_array, dict_ORCA, boundary)
            if len(expanded_grid_array) == 0:
                robot.logger.info("invalid even after trying expanded grid")
                invalid = 1
            else:
                robot.logger.info("was invalid but found in expanded grid")
                invalid = 0
                orca_grid_array = expanded_orca_grid_array
            # robot.set_led(255,255,255)
        if invalid == 0:
            [x_dot,y_dot] = choose_orca_vel([x_dot,y_dot], orca_grid_array)
            robot.logger.info(str(robot.id) + " : output: [" + str(x_dot) + ", " + str(y_dot) + "]")
            invalid = 0
        # within_other_robot_dist_flag = 0
        
        curr_pose = robot.get_pose()
        while not curr_pose:
            curr_pose = robot.get_pose()
        curr_x = curr_pose[0]
        curr_y = curr_pose[1]
        curr_theta = curr_pose[2]
        second_position_timestamp = robot.get_clock()

        X_latest = (curr_x + D*math.cos(curr_theta))
        Y_latest = (curr_y + D*math.sin(curr_theta))  
        
        dist = math.sqrt((X_latest-desired_x)**2 + (Y_latest-desired_y)**2)

        if invalid == 1:
            robot.set_led(255,0,0)
            u = [0,0]
            effective_x_dot = 0
            effective_y_dot = 0
        elif dist < final_tolerance:
            robot.set_led(0,255,0)
            u = [0,0]
            effective_x_dot = 0
            effective_y_dot = 0
            robot.send_msg(struct.pack('ifffff', robot.id, curr_x, curr_y, curr_theta, 0, 0))
            turn_to_angle(robot, curr_theta, final_theta)
            robot.send_msg(struct.pack('ifffff', robot.id, curr_x, curr_y, curr_theta, 0, 0))
            robot.set_led(100,0,200)
            robot.logger.info("finished at: " + str(robot.get_clock()))
            # desired_point_flag = desired_point_flag * -1
            waypoint_index = (waypoint_index+1) % 4
            return
        else:
            robot.set_led(0,0,255)
            robot.logger.info("calculating wheel speeds")
            theta = curr_theta
            M = np.array([[((math.cos(theta)/2)+(D*math.sin(theta)/L)), ((math.cos(theta)/2)-(D*math.sin(theta)/L))],[((math.sin(theta)/2)-(D*math.cos(theta)/L)), ((math.sin(theta)/2)+(D*math.cos(theta)/L))]])
            M_inv = np.linalg.inv(M)
            angle_error = desired_theta-curr_theta

            # robot.logger.info("xdot: " + str(x_dot) + " ydot: " + str(y_dot))
            effective_x_dot = x_dot # + angle_error*P_gain # - D*math.sin(curr_theta)*theta_dot
            effective_y_dot = y_dot # + angle_error*P_gain # + D*math.cos(curr_theta)*theta_dot
            u,position_filter_prev,integrated_error,speed_filter = calculate_wheel_speeds(robot, position_filter_prev, [effective_x_dot,effective_y_dot], curr_pose, prev_pose, (second_position_timestamp-first_position_timestamp), integrated_error, speed_filter)

        robot.set_vel(u[0],u[1])
        if square == 1:
            angle_step_count = angle_step_count+1
        else:
            angle_index = (angle_index+1)%(len(desired_angle_array))

def compute_own_velocity(desired, current):
    own_vopt_x = desired[0]-current[0]
    own_vopt_y = desired[1]-current[1]

    opt_theta = math.atan2(own_vopt_y, own_vopt_x)
    return own_vopt_x, own_vopt_y, opt_theta

def unpack_other_pos_string(robot, str):
    # robot.logger.info(len(str))
    string_return = []
    i = 0
    while i < (len(str)):
    # robot.logger.info(str[0])
        temp = struct.unpack('ifffff',str[i][:24])
        j = 0
        while j < len(temp):
            string_return.append(temp[j])
            j = j+1
        i = i +1
    return string_return

def compute_Vo(robot, own_pos, other_pos):
    own_x = own_pos[0]
    own_y = own_pos[1]
    other_x = other_pos[0]
    other_y = other_pos[1]
    
    # finding center point of the two tangent lines
    x2 = other_x - own_x
    y2 = other_y - own_y
    x1 = x2 / time_step
    y1 = y2 / time_step
    r2 = 2 * effective_robot_radius
    r1 = r2 / time_step

    dx, dy = -x1, -y1
    dxr, dyr = -dy, dx
    d = math.sqrt(dx ** 2 + dy ** 2)
    # # robot.logger.info('%f %f' %(d, r1))
    if d < r1:
        d = r1+0.00000000000001

    rho = r1 / d
    ad = rho ** 2
    bd = rho * math.sqrt(1 - rho ** 2)
    T1x = x1 + ad * dx + bd * dxr
    T1y = y1 + ad * dy + bd * dyr
    T2x = x1 + ad * dx - bd * dxr
    T2y = y1 + ad * dy - bd * dyr
    # # robot.logger.info(T1x)

    m1 = T1y / T1x
    m2 = T2y / T2x
    den = (T2x - T1x)
    if den == 0:
        den = 0.0000001
    m3 = ((T2y - T1y) / den)
    b3 = T1y - (m3 * T1x)
    cx = x1
    cy = y1
    cr = r1

    if x2 == 0:
        x2 = 0.00000000000001
    slope_c2 = y2/x2
    a = 1+slope_c2**2
    b = -2*x2 - 2*slope_c2*y2
    c = x2**2+y2**2-r2**2
    # print(b**2-4*a*c)
    intersect_x_pos = (-b+math.sqrt(b**2-4*a*c))/(2*a)
    intersect_y_pos = slope_c2*intersect_x_pos
    intersect_x_neg = (-b-math.sqrt(b**2-4*a*c))/(2*a)
    intersect_y_neg = slope_c2*intersect_x_neg
    # print("%f, %f %f, %f" %(intersect_x_pos,intersect_y_pos, intersect_x_neg, intersect_y_neg))
    dist_pos = math.sqrt((intersect_x_pos-x1)**2 + (intersect_y_pos-y1)**2)
    dist_neg = math.sqrt((intersect_x_neg-x1)**2 + (intersect_y_neg-y1)**2)

    if dist_pos < dist_neg:
      intersect_x = intersect_x_neg
      intersect_y = intersect_y_neg
    else:
      intersect_x = intersect_x_pos
      intersect_y = intersect_y_pos

    m4 = -x2/y2
    b4 = intersect_y-m4*intersect_x
    return [m1, m2, m3, b3, cx, cy, cr, T1x, T1y, T2x, T2y, m4, b4]

def compute_w_old(robot, vo, own_vopt_x, own_vopt_y, other_vopt_x, other_vopt_y):
    import math
    # import sympy as sym
    # # robot.logger.info('here')

    # calculate va-vb optimal
    v_opt_x = own_vopt_x-other_vopt_x
    v_opt_y = own_vopt_y-other_vopt_y
    #   # robot.logger.info("v_opt: %f %f" %(v_opt_x,v_opt_y))

    x_res_plus = ((vo[6]*(vo[4]-v_opt_x))/math.sqrt(vo[4]**2 - 2*vo[4]*v_opt_x + vo[5]**2 - 2*vo[5]*v_opt_y + v_opt_x**2 + v_opt_y**2)) + vo[4]
    x_res_minus = -((vo[6]*(vo[4]-v_opt_x))/math.sqrt(vo[4]**2 - 2*vo[4]*v_opt_x + vo[5]**2 - 2*vo[5]*v_opt_y + v_opt_x**2 + v_opt_y**2)) + vo[4]
    c_x_plus = x_res_plus
    c_y_plus = (math.sqrt(vo[6]**2 - (x_res_plus-vo[4])**2)) + vo[5]
    c_dist_plus = math.sqrt((c_x_plus - v_opt_x) ** 2 + (c_y_plus - v_opt_y) ** 2)
    c_x_minus = x_res_minus
    try: 
        c_y_minus = (math.sqrt(vo[6]**2 - (x_res_minus-vo[4])**2)) + vo[5]
        c_dist_minus = math.sqrt((c_x_minus - v_opt_x) ** 2 + (c_y_minus - v_opt_y) ** 2)
        if c_dist_plus > c_dist_minus:
            closest_dist = c_dist_minus
            closest_x = c_x_minus
            closest_y = c_y_minus
        else:
            closest_dist = c_dist_plus
            closest_x = c_x_plus
            closest_y = c_y_plus
    except:
        closest_dist = c_dist_plus
        closest_x = c_x_plus
        closest_y = c_y_plus

    #   # robot.logger.info("to circle: %f %f" %(closest_x,closest_y))

    x_res = (v_opt_x + vo[0]*v_opt_y)/(1+vo[0]**2)
    tan1_x = x_res
    tan1_y = (vo[0]*x_res)
    tan1_dist = math.sqrt((tan1_x-v_opt_x)**2+(tan1_y-v_opt_y)**2)
    #   # robot.logger.info("to line 1: %f %f" %(tan1_x,tan1_y))

    # if ((v_opt_x - vo[4])**2 + (v_opt_y - vo[5])**2 < vo[6]**2):
    y_m3 = vo[2] * closest_x  + vo[3]
    y_m3_center = vo[2] * vo[4] + vo[3]
    if ((vo[5] <= y_m3_center and closest_y <= y_m3) or (vo[5] >= y_m3_center and closest_y >= y_m3)):
        # # robot.logger.info("on boundary side of circle")
        closest_dist = tan1_dist
        closest_x = tan1_x
        closest_y = tan1_y

    x_res = (v_opt_x + vo[1]*v_opt_y)/(1+vo[1]**2)
    tan2_x = x_res
    tan2_y = (vo[1] * x_res)
    tan2_dist = math.sqrt((tan2_x - v_opt_x) ** 2 + (tan2_y - v_opt_y) ** 2)
    
    #   # robot.logger.info("to line 2: %f %f" %(tan2_x,tan2_y))

    y_m3 = vo[2] * tan1_x + vo[3]
    y_m3_center = vo[2] * vo[4] + vo[3]
    if ((vo[5] <= y_m3_center and tan1_y <= y_m3) or (vo[5] >= y_m3_center and tan1_y >= y_m3)):
        # # robot.logger.info("on boundary side t1")
        if tan1_dist < closest_dist:
            closest_dist = tan1_dist
            closest_x = tan1_x
            closest_y = tan1_y
    
    y_m3 = vo[2] * tan2_x + vo[3]
    y_m3_center = vo[2] * vo[4] + vo[3]
    if ((vo[5] <= y_m3_center and tan2_y <= y_m3) or (vo[5] >= y_m3_center and tan2_y >= y_m3)):
        # # robot.logger.info("on boundary side t2")
        if tan2_dist < closest_dist:
            closest_dist = tan2_dist
            closest_x = tan2_x
            closest_y = tan2_y
    # # robot.logger.info(closest_x, closest_y)
    return [(closest_x-v_opt_x),(closest_y-v_opt_y)]

def compute_w(robot, vo, own_vopt, other_vopt):
    # assuming a time step of > 1
    # calculate va-vb
    vel_diff = [own_vopt[0]-other_vopt[0], own_vopt[1]-other_vopt[1]]
    # # compute what side of the 'divided by timestep' circle's line the cut-off parallel line is using the same method in the grid update function
    # the points we can use to construct this line are its x and y intercepts
    pt_1 = [0,vo[3]]
    x = -vo[3]/vo[2]
    pt_2 = [x,0]
    check_point = [0, vo[12]]
    side_of_line = (check_point[0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (check_point[1]-pt_1[1])*(pt_2[0]-pt_1[0])
    # # calculate vel_diff distance to line 1
    # since shortest distance is a perpendicular line, form the line that is perpendicular and passes through the vel_diff line
    perpendicular_slope = -1/vo[0]
    perpendicular_intercept = vel_diff[1]-(perpendicular_slope*vel_diff[0])
    x_check_1 = (perpendicular_intercept-0)/(vo[0]-perpendicular_slope)
    y_check_1 = vo[0]*x_check_1

    check_line_side = (x_check_1-pt_1[0])*(pt_2[1]-pt_1[1]) - (y_check_1-pt_1[1])*(pt_2[0]-pt_1[0])
    # if not on velocity obstacle side, discard point
    if np.sign(check_line_side) == np.sign(side_of_line) or np.sign(check_line_side) == 0:
        invalid_check_1 = 0
        dist_check_1 = math.sqrt((vel_diff[0]-x_check_1)**2 + (vel_diff[1]-y_check_1)**2)
    else:
        invalid_check_1 = 1
    # repeat for line 2
    perpendicular_slope = -1/vo[1]
    perpendicular_intercept = vel_diff[1]-(perpendicular_slope*vel_diff[0])
    x_check_2 = (perpendicular_intercept-0)/(vo[1]-perpendicular_slope)
    y_check_2 = vo[1]*x_check_2

    check_line_side = (x_check_2-pt_1[0])*(pt_2[1]-pt_1[1]) - (y_check_2-pt_1[1])*(pt_2[0]-pt_1[0])
    # if not on velocity obstacle side, discard point
    if np.sign(check_line_side) == np.sign(side_of_line) or np.sign(check_line_side) == 0:
        invalid_check_2 = 0
        dist_check_2 = math.sqrt((vel_diff[0]-x_check_2)**2 + (vel_diff[1]-y_check_2)**2)
    else:
        invalid_check_2 = 1
    # # calculate distance to circle
    # line from center of circle to velocity difference point
    slope = (vo[5]-vel_diff[1])/(vo[4]-vel_diff[0])
    intercept = vo[5]-(slope*vo[4])

    # using quadratic formula to get 2 points where line intersects circle
    a_quadratic_term = (1+slope**2)
    b_quadratic_term = (2*slope*intercept)-(2*slope*vo[5])-(2*vo[4])
    c_quadratic_term = -(vo[6]**2-vo[4]**2-vo[5]**2-intercept**2+(2*intercept*vo[5]))

    # print(b_quadratic_term**2-(4*a_quadratic_term*c_quadratic_term))
    x1 = (-b_quadratic_term+math.sqrt(b_quadratic_term**2-(4*a_quadratic_term*c_quadratic_term)))/(2*a_quadratic_term)
    x2 = (-b_quadratic_term-math.sqrt(b_quadratic_term**2-(4*a_quadratic_term*c_quadratic_term)))/(2*a_quadratic_term)
    y1 = slope*x1+intercept
    y2 = slope*x2+intercept

    dist_check_circle_1 = math.sqrt((vel_diff[0]-x1)**2 + (vel_diff[1]-y1)**2)
    dist_check_circle_2 = math.sqrt((vel_diff[0]-x2)**2 + (vel_diff[1]-y2)**2)
    # will get 2 points from this, choose closer one
    if if_valid_velocity(vo[0], vo[1], vo[2], vo[3], vo[4], vo[5], vo[6], vo[7], vo[8], vo[9], vo[10], vel_diff[0], vel_diff[1]):
        if dist_check_circle_1 > dist_check_circle_2:
            closest_x = x1
            closest_y = y1
            closest_dist = dist_check_circle_1
        else:
            closest_x = x2
            closest_y = y2
            closest_dist = dist_check_circle_2
    else:
        if dist_check_circle_1 < dist_check_circle_2:
            closest_x = x1
            closest_y = y1
            closest_dist = dist_check_circle_1
        else:
            closest_x = x2
            closest_y = y2
            closest_dist = dist_check_circle_2

    # calculate distance to outer vo line (cutoff line)
    perpendicular_slope = -1/vo[11]
    perpendicular_intercept = vel_diff[1]-(perpendicular_slope*vel_diff[0])
    x_check_3 = (perpendicular_intercept-vo[12])/(vo[11]-perpendicular_slope)
    y_check_3 = vo[11]*x_check_3 + vo[12]

    dist_check_3 = math.sqrt((vel_diff[0]-x_check_3)**2 + (vel_diff[1]-y_check_3)**2)
    if if_valid_velocity(vo[0], vo[1], vo[2], vo[3], vo[4], vo[5], vo[6], vo[7], vo[8], vo[9], vo[10], x_check_3, y_check_3):
        invalid_check_3 = 0
    else:
        invalid_check_3 = 1
    # print(str(x_check_3) + ", " + str(y_check_3))
    # check if point is outside velocity cutoff line
    pt_1 = [0,vo[12]]
    x = -vo[12]/vo[11]
    pt_2 = [x,0]
    check_point = [0, vo[3]]
    side_of_line = (check_point[0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (check_point[1]-pt_1[1])*(pt_2[0]-pt_1[0])
    
    check_line_side = (vel_diff[0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (vel_diff[1]-pt_1[1])*(pt_2[0]-pt_1[0])
    if np.sign(check_line_side) == np.sign(side_of_line): # or np.sign(check_line_side) == 0:
        beyond_vo = 0
    else:
        beyond_vo = 1

    # choose closest point of 4 options  
    if invalid_check_1 == 0: # and beyond_vo == 0:
        if dist_check_1 < closest_dist:
            closest_x = x_check_1
            closest_y = y_check_1
            closest_dist = dist_check_1
    if invalid_check_2 == 0: # and beyond_vo == 0:
        if dist_check_2 < closest_dist:
            closest_x = x_check_2
            closest_y = y_check_2
            closest_dist = dist_check_2
    # if invalid_check_3 == 0 and beyond_vo == 1:
    #     if dist_check_3 < closest_dist:
    #         closest_x = x_check_3
    #         closest_y = y_check_3
    #         closest_dist = dist_check_3
    # return closest_point - (va-vb)_point
    # print(str(closest_x) + ", " + str(closest_y))
    w = [(closest_x-vel_diff[0]),(closest_y-vel_diff[1])]
    return w

#checks if inside boudaries and returns true if it is in the velocity obstacle 
def if_valid_velocity(m1, m2, m3, b3, cx, cy, cr, T1x, T1y, T2x, T2y, x_check, y_check):
    import math
    ang1 = (math.atan2(T1y, T1x) + (2*math.pi)) % (2*math.pi)
    ang2 = (math.atan2(T2y, T2x) + (2*math.pi)) % (2*math.pi)
    ang_check = (math.atan2(y_check,x_check) + (2*math.pi)) % (2*math.pi)
    if (abs(ang1-ang2)<math.pi):
        if (ang_check <= ang1 and ang_check >= ang2) or (ang_check >= ang1 and ang_check <= ang2):
            if (((x_check-cx)**2 + (y_check-cy)**2) <= cr**2):
                return True
            y_m3 = m3*x_check + b3
            if (0 <= b3 and y_check >= y_m3) or (0 >= b3 and y_check <= y_m3):
                return True
    else:
        if (ang1>ang2):
            if (ang_check >= ang1 or ang_check <= ang2):
                if (((x_check-cx)**2 + (y_check-cy)**2) <= cr**2):
                    return True
                y_m3 = m3*x_check + b3
                if (0 <= b3 and y_check >= y_m3) or (0 >= b3 and y_check <= y_m3):
                    return True
        elif (ang2>ang1):
            if (ang_check > ang2 or ang_check < ang1):
                if (((x_check-cx)**2 + (y_check-cy)**2) <= cr**2):
                    return True
                y_m3 = m3*x_check + b3
                if (0 <= b3 and y_check >= y_m3) or (0 >= b3 and y_check <= y_m3):
                    return True
    return False
    
def compute_n(vo, w, own_vopt_x, own_vopt_y, other_vopt_x, other_vopt_y):
    w_angle = math.atan2(w[1],w[0])
    vel_diff = [own_vopt_x-other_vopt_x, own_vopt_y-other_vopt_y]
    if if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],vel_diff[0],vel_diff[1]):
        # pt_1 = [0,vo[12]]
        # x = -vo[12]/vo[11]
        # pt_2 = [x,0]
        # check_point = [0, vo[3]]
        # side_of_line = (check_point[0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (check_point[1]-pt_1[1])*(pt_2[0]-pt_1[0])
        
        # check_line_side = (vel_diff[0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (vel_diff[1]-pt_1[1])*(pt_2[0]-pt_1[0])
        # if np.sign(check_line_side) == np.sign(side_of_line): # or np.sign(check_line_side) == 0:
        #     beyond_vo = 0
        # else:
        #     beyond_vo = 1

        # if beyond_vo == 0:
        print("in velocity obstacle")
        return [math.cos(w_angle), math.sin(w_angle)]
    # else:
    if w_angle < 0:
        w_angle = w_angle%(math.pi)
    else:
        w_angle = w_angle-(math.pi)
    return [math.cos(w_angle), math.sin(w_angle)]

def gen_ORCA_line(robot, vo, n, w, own_vopt_x, own_vopt_y):
    try:
            slope = -n[0]/n[1]
    except: 
        slope = -n[0]/0.00001
    intercept = (own_vopt_y+responsibility*w[1]) + (-slope)*(own_vopt_x+responsibility*w[0])
    check_x = vo[4]
    check_y = slope*vo[4]+intercept
    check_y_2 = vo[5]
    check_x_2 = (vo[5]-intercept)/slope
    # robot.logger.info("to check ORCA: %f, %f and %f, %f" %(check_x, check_y, check_x_2, check_y_2))
    if if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],check_x, check_y) or if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],check_x_2, check_y_2):
        # robot.logger.info("flipping intercept circle")
        intercept = -intercept
    elif abs(slope - vo[0]) < 0.00001:  
        # robot.logger.info("parallel to vo[0] %f %f" %(vo[1], slope))
        if vo[1] == slope:
            slope = slope + 0.000001
        intersect_x = intercept/(vo[1]-slope)
        intersect_y = vo[1]*intersect_x
        # # robot.logger.info(intersect_x, intersect_y)
        if if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],intersect_x, intersect_y):
            # robot.logger.info("flipping intercept m1")
            intercept = -intercept 
    elif abs(slope - vo[1]) < 0.00001:  
        if vo[1] == slope:
            slope = slope + 0.000001  
        # robot.logger.info("parallel to vo[1] %f %f" %(vo[0], slope))
        intersect_x = intercept/(vo[0]-slope)
        intersect_y = vo[0]*intersect_x
        # # robot.logger.info(intersect_x, intersect_y)
        if if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],intersect_x, intersect_y):
            # robot.logger.info("flipping intercept m2")
            intercept = -intercept 
    return [slope, intercept]

def gen_ORCA_line_new(robot, vo, n, w, own_vopt_x, own_vopt_y):
    try:
        slope = -n[0]/n[1]
    except: 
        slope = -n[0]/0.00001

    intercept = (own_vopt_y+responsibility*w[1]) - (slope*(own_vopt_x+responsibility*w[0]))
    return [slope, intercept]

# combining ORCAs
def combine_ORCAs(robot, opt_x, opt_y, dict_ORCA, dict_vo, checking_id):
    found = 0
    if len(dict_ORCA) == 1:
        # robot.logger.info('only 1  in dict')
        return get_ORCA_vel(dict_vo[checking_id], opt_x, opt_y, dict_ORCA[checking_id])
    if len(dict_ORCA) > 1:
        if if_valid_velocity(dict_vo[checking_id][0],dict_vo[checking_id][1],dict_vo[checking_id][2],dict_vo[checking_id][3],
                        dict_vo[checking_id][4],dict_vo[checking_id][5],dict_vo[checking_id][6],dict_vo[checking_id][7],dict_vo[checking_id][8],
                        dict_vo[checking_id][9],dict_vo[checking_id][10],opt_x, opt_y):
            # robot.logger.info("in collision path")
            for key in dict_ORCA:
                if key != checking_id:
                    den = dict_ORCA[checking_id][0]-dict_ORCA[key][0]
                    if den == 0:
                        den = 0.000001
                    intersect_x = (dict_ORCA[key][1]-dict_ORCA[checking_id][1])/(den)
                    intersect_y = dict_ORCA[checking_id][0]*intersect_x + dict_ORCA[checking_id][1]
                    invalid = 0
                    for inner_key in dict_ORCA:
                        if inner_key != checking_id:
                            if if_valid_velocity(dict_vo[inner_key][0],dict_vo[inner_key][1],dict_vo[inner_key][2],dict_vo[inner_key][3],
                                        dict_vo[inner_key][4],dict_vo[inner_key][5],dict_vo[inner_key][6],dict_vo[inner_key][7],dict_vo[inner_key][8],
                                        dict_vo[inner_key][9],dict_vo[inner_key][10],intersect_x, intersect_y):
                                invalid = 1
                    if invalid == 0:
                        if found == 0:
                            ideal_x = intersect_x
                            ideal_y = intersect_y
                            found = 1
                        else:
                            dist_intersect = math.sqrt((intersect_x - opt_x) ** 2 + (intersect_y - opt_y) ** 2)
                            dist_ideal = math.sqrt((ideal_x - opt_x) ** 2 + (ideal_y - opt_y) ** 2)
                            if dist_intersect < dist_ideal:
                                ideal_x = intersect_x
                                ideal_y = intersect_y
            if found == 1:
                return [ideal_x, ideal_y]
            else:
                # robot.logger.info("no valid options")
                return [0,0]
        else:
            # return [opt_x,opt_y]
            return get_ORCA_vel(dict_vo[checking_id], opt_x, opt_y, dict_ORCA[checking_id])

def get_ORCA_vel(vo, own_vopt_x, own_vopt_y, ORCA_line):
#   if if_valid_velocity(vo[0],vo[1],vo[2],vo[3],vo[4],vo[5],vo[6],vo[7],vo[8],vo[9],vo[10],own_vopt_x, own_vopt_y):
    closest_x = (-ORCA_line[1]*ORCA_line[0] + ORCA_line[0]*own_vopt_y + own_vopt_x)/(ORCA_line[0]**2+1)
    closest_y = ORCA_line[0]*closest_x+ORCA_line[1]
    return [closest_x, closest_y]
#   else:
#     return [own_vopt_x, own_vopt_y]

def get_indv_position_filter(robot, curr_pose, avg_theta,position_filter):
    wheel_dist = 0.35
    vive_loc = np.array([curr_pose[0]-wheel_dist*math.sin(avg_theta),curr_pose[1]+wheel_dist*math.cos(avg_theta),curr_pose[0]+wheel_dist*math.sin(avg_theta),curr_pose[1]-wheel_dist*math.cos(avg_theta)])
    # robot.logger.info(str(vive_loc))
    position_filter = 0.8*position_filter + 0.2*vive_loc
    return position_filter

def calculate_wheel_speeds(robot, position_filter_prev, desired_vel, curr_pose, prev_pose, vive_update_rate, integrated_error, speed_filter):
    position_filter_prev = get_indv_position_filter(robot,prev_pose, prev_pose[2], position_filter_prev)
    position_filter = get_indv_position_filter(robot,curr_pose, curr_pose[2], position_filter_prev)    
    
    actual_vel_x = (curr_pose[0]-prev_pose[0])/vive_update_rate
    actual_vel_y = (curr_pose[1]-prev_pose[1])/vive_update_rate

    actual_vel_x_left = (position_filter[0]-position_filter_prev[0])/vive_update_rate
    actual_vel_y_left = (position_filter[1]-position_filter_prev[1])/vive_update_rate

    actual_vel_x_right = (position_filter[2]-position_filter_prev[2])/vive_update_rate
    actual_vel_y_right = (position_filter[3]-position_filter_prev[3])/vive_update_rate

    error_x = desired_vel[0] - actual_vel_x
    error_y = desired_vel[1] - actual_vel_y
    robot.logger.info(str(robot.get_clock()) + "," + str(desired_vel[0]) + "," + str(desired_vel[1]) + "," + str(actual_vel_x) + "," + str(actual_vel_y)) # + "," + str(curr_pose[0]) + "," + str(curr_pose[1])+ "," + str(error_x) + "," + str(error_y))

    integrated_error[0] = integrated_error[0] + (error_x*I_gain)
    integrated_error[1] = integrated_error[0] + (error_y*I_gain)
    if abs(integrated_error[0]) > I_windup_max:
        integrated_error[0] = I_windup_max * np.sign(integrated_error[0])
    if abs(integrated_error[1]) > I_windup_max:
        integrated_error[1] = I_windup_max * np.sign(integrated_error[1])

    x_vel = desired_vel[0]
    y_vel = desired_vel[1]
    
    theta = curr_pose[2]
    M = np.array([[((math.cos(theta)/2)+(D*math.sin(theta)/L)), ((math.cos(theta)/2)-(D*math.sin(theta)/L))],[((math.sin(theta)/2)-(D*math.cos(theta)/L)), ((math.sin(theta)/2)+(D*math.cos(theta)/L))]])
    M_inv = np.linalg.inv(M)
    u = M_inv.dot([x_vel,y_vel])
    u_wheel_mid = M_inv.dot([x_vel + (error_x*P_gain),y_vel + (error_y*P_gain)])

    actual_speed_left = math.sqrt(actual_vel_x_left**2 + actual_vel_y_left**2)
    speed_filter[0] = 0.9*speed_filter[0] + 0.1*actual_speed_left
    
    error_left = abs(u[0]) - speed_filter[0] 

    actual_speed_right = math.sqrt(actual_vel_x_right**2 + actual_vel_y_right**2)
    speed_filter[1] = 0.9*speed_filter[1] + 0.1*actual_speed_right
    
    error_right = abs(u[1]) - speed_filter[1] 
    robot.logger.info("here")

    integrated_error[2] = integrated_error[2] + (error_left*I_gain)
    # integrated_error_left_y = integrated_error_left_y + (error_left*I_gain)
    if abs(integrated_error[2]) > I_windup_max:
        integrated_error[2] = I_windup_max * np.sign(integrated_error[2])
    # robot.logger.info(str(desired_speed_left) + "," + str(speed_filter_left) + "," + str(integrated_error_left))

    integrated_error[3] = integrated_error[3] + (error_right*I_gain)
    # integrated_error_right_y = integrated_error_right_y + (error_left*I_gain)
    if abs(integrated_error[3]) > I_windup_max:
        integrated_error[3] = I_windup_max * np.sign(integrated_error[3])
    # robot.logger.info(str(desired_speed_right) + "," + str(speed_filter_right) + "," + str(integrated_error_right))

    # robot.set_vel(u[0],u[1])
    # robot.set_vel(0,0)

    vl = u_wheel_mid[0] + (integrated_error[2] + error_left*P_gain_speed) * np.sign(u[0])
    if abs(vl) > max_wheel_speed:
        vl = max_wheel_speed * np.sign(vl)
    elif abs(vl) < min_wheel_speed:
        vl = min_wheel_speed * np.sign(vl)
    vl = math.floor(vl)

    vr = u_wheel_mid[1] + (integrated_error[3] + error_right*P_gain_speed) * np.sign(u[1])
    if abs(vr) > max_wheel_speed:
        vr = max_wheel_speed * np.sign(vr)
    elif abs(vr) < min_wheel_speed:
        vr = min_wheel_speed * np.sign(vr)
    vr = math.floor(vr)

    robot.logger.info(str(robot.id) + " : new wheel speeds " + str(vl) + " , " + str(vr))
    return [vl,vr], position_filter_prev, integrated_error, speed_filter

def generate_grid(grid_resolution, grid_x_bound, grid_y_bound):
    grid_array = {}
    index = 0

    for x_tick in np.arange(-grid_x_bound, grid_x_bound, grid_resolution):
        for y_tick in np.arange(-grid_y_bound, grid_y_bound, grid_resolution):
            grid_array[index] = [x_tick,y_tick,1]
            index = index + 1
    return grid_array

# https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
# grid array is a dictionary of grid positions and a validity flag
# ORCA_dict is a dictionary containing the slope and intercept of the orca line with that robot
# boundary is a dictionary containing the geometric parameters to construct the velocity obstacle with that robot -> [m1, m2, m3, b3, cx, cy, cr, T1x, T1y, T2x, T2y, m4, b4]
def update_grid(grid_array, ORCA_dict, boundary):
    for bot in ORCA_dict:
        pt_1 = [0, ORCA_dict[bot][1]]
        # calculate x intercept
        x = -ORCA_dict[bot][1]/ORCA_dict[bot][0]
        pt_2 = [x,0]
        # calculate which side of the orca line the center of the vo area is
        d = (boundary[bot][4]-pt_1[0])*(pt_2[1]-pt_1[1]) - (boundary[bot][5]-pt_1[1])*(pt_2[0]-pt_1[0])
        remove_indexes = []
        for index in grid_array:
            if grid_array[index][2] == 1:
                d_grid = (grid_array[index][0]-pt_1[0])*(pt_2[1]-pt_1[1]) - (grid_array[index][1]-pt_1[1])*(pt_2[0]-pt_1[0])
                # compute if it is on the vo side of the line
                if np.sign(d) == np.sign(d_grid):
                    # if so, remove that spot from the grid_array dictionary
                    remove_indexes.append(index)
        for ind in remove_indexes:
            grid_array.pop(ind)
    return grid_array

def choose_orca_vel(opt_vel, orca_grid):
    closest_dist = 100000
    orca_vel = []
    for point in orca_grid:
        dist = math.sqrt((orca_grid[point][0]-opt_vel[0])**2 + (orca_grid[point][1]-opt_vel[1])**2)
        if dist < closest_dist:
            closest_dist = dist
            orca_vel = [orca_grid[point][0],orca_grid[point][1]]
    return orca_vel

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

def set_desired_pose(id):
    constant_square = 0.2
    if id == 11:
        return [-constant_square,-constant_square,math.pi/4]
    if id == 12:
        return [-constant_square,constant_square,-math.pi/4]
    if id == 13:
        return [constant_square,constant_square,-math.pi*0.75]
    if id == 14:
        return [constant_square,-constant_square,math.pi*0.75]
    if id == 11:
        return [constant_square,constant_square,-math.pi*0.75]
    if id == 12:
        return [constant_square,-constant_square,math.pi*0.75]
    if id == 13:
        return [-constant_square,-constant_square,math.pi/4]
    if id == 14:
        return [-constant_square,constant_square,-math.pi/4]

    if id == 40:
        return [-0.3,-0.3,0]
    if id == 45:
        return [0.3,0.3,0]
    # return [(id/10), 0, 0]

def N_init_positions_skinny(robot,robot_id):
    height = 10
    between_each = 0.32
    init_pose = {}
    init_count = 9
    count = init_count
    end_angle = 0

    init_pose[init_count] = [-1.3,-1.2,end_angle]
    count = count + 1

    x = init_pose[init_count][0]
    y = init_pose[init_count][1]

    # robot.logger.info("in N")
    # column 1
    for j in range(height-1):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1

    # column 2
    x = x+between_each
    y = init_pose[init_count][1]-between_each
    for k in range(height):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1
    
    # column 3
    x = x+between_each
    y = init_pose[init_count][1]-between_each
    for k in range(height):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1

    # diagonal
    for i in range(height/2):
        x = x+(between_each)
        init_pose[count] =[x,y]
        count = count + 1
        y = y-(between_each*1.5)
        init_pose[count] =[x,y,end_angle]
        count = count + 1       

    # extra part of N
    y = init_pose[init_count][1] + ((height-1)*between_each)
    for c in range(height/2):
        init_pose[count] = [x,y,end_angle]
        y = y - between_each
        count = count+1

    # end column left
    x = x + between_each
    y = init_pose[init_count][1]
    for a in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    #end column middle
    x = x + between_each
    y = init_pose[init_count][1]
    for b in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    #end column right
    x = x + between_each
    y = init_pose[init_count][1]
    for b in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    # side bar 
    x = init_pose[init_count][0]+between_each*3
    y = init_pose[init_count][1]
    for c in range(height/2):
        init_pose[count] = [x,y,end_angle]
        y = y + between_each
        count = count+1

    for pose in init_pose:
        init_pose[pose].append(math.pi)
        
    # print(count-1)
    if robot_id in init_pose:
        return init_pose[robot_id]
    else:
        return False

def N_init_positions(robot,robot_id):
    height = 5
    between_each = 0.32
    init_pose = {}
    init_count = 0
    count = init_count
    end_angle = 0

    init_pose[init_count] = [-1.2,-0.8,end_angle]
    count = count + 1

    x = init_pose[init_count][0]
    y = init_pose[init_count][1]

    # robot.logger.info("in N")
    # column 1
    for j in range(height-1):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1

    # column 2
    x = x+between_each
    y = init_pose[init_count][1]-between_each
    for k in range(height):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1
    
    # column 3
    x = x+between_each
    y = init_pose[init_count][1]-between_each
    for k in range(height):
        y = y+between_each
        init_pose[count] = [x,y,end_angle]
        count = count + 1

    # diagonal
    for i in range(height):
        x = x+between_each
        init_pose[count] =[x,y,end_angle]
        count = count + 1
        y = y-between_each
        init_pose[count] =[x,y,end_angle]
        count = count + 1       

    # extra part of N
    y = init_pose[init_count][1] + ((height-1)*between_each)
    for c in range(height/2):
        init_pose[count] = [x,y,end_angle]
        y = y - between_each
        count = count+1

    # end column left
    x = x + between_each
    y = init_pose[init_count][1]
    for a in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    #end column middle
    x = x + between_each
    y = init_pose[init_count][1]
    for b in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    #end column right
    x = x + between_each
    y = init_pose[init_count][1]
    for b in range(height):
        init_pose[count] = [x,y,end_angle]
        y = y+between_each
        count = count + 1

    # side bar 
    x = init_pose[init_count][0]+between_each*3
    y = init_pose[init_count][1]
    for c in range(height/2):
        init_pose[count] = [x,y,end_angle]
        y = y + between_each
        count = count+1

    # print(count-1)
    if robot_id in init_pose:
        return init_pose[robot_id]
    else:
        return False