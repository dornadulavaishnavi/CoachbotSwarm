import math
import struct
def usr(robot):
    robot.delay(3000)
    desired_distance = .5 #will vary from 0.3-0.5
    log = open("experiment_log", "w")
    log.write("Experiment start\n")
    log.flush()
    # initialize variables
    current_distance = None
    pose_robot0 = []
    default_speed = 10
    Kp = 15
    Ki = 0
    Kd = 100
    error_prev = 0
    error_sum = 0
    time_prev = 0
    current_time = robot.get_clock()

    while True:
        robot.delay()
        if (robot.virtual_id() == 0):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                pose_robot1 = [float(pose_rxed[0]), float(pose_rxed[1])]
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                #print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.virtual_id()))  # send pose x,y in message

            pose_robot0 = pose_t
            try:
                #blink based on the distance
                if len(msgs) > 0:
                    current_distance = math.sqrt((pose_robot0[0] - pose_robot1[0])**2 + (pose_robot0[1] - pose_robot1[1])**2)
                    if current_distance > desired_distance:
                        robot.set_led(100,0,0)
                        robot.delay(10)
                        robot.set_led(0,0,0)
                    else:
                        robot.set_led(0,100,0)
                        robot.delay(10)
                        robot.set_led(0,0,0)
            except Exception as e:
                log.write(f"An error occurred: {e}")
                log.flush()
            #robot.set_vel(5, 5)
         
         
        if (robot.id == 1):
            # if we received a message, print out info in message
            pose_robot0 = []
            output = 0
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])
                pose_robot0 = [float(pose_rxed[0]), float(pose_rxed[1])]

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                #print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1], pose_t[2])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.virtual_id()))  # send pose x,y in message

            pose_robot1 = pose_t

            #try this and print out the error
            try:
                if len(msgs) > 0:
                    # blink based on the distance
                    current_distance = math.sqrt((pose_robot0[0] - pose_robot1[0])**2 + (pose_robot0[1] - pose_robot1[1])**2)
                    # print("*******************start**************************")
                    if current_distance > desired_distance:
                        robot.set_led(100,0,0)
                        robot.delay(10)
                        robot.set_led(0,0,0)
                    else:
                        robot.set_led(0,100,0)
                        robot.delay(10)
                        robot.set_led(0,0,0)

                    log.write("current distance is: ", current_distance)
                    log.flush()

                    # PID controller 
                    time_prev = current_time
                    current_time = robot.get_clock()
                    dt = current_time - time_prev
                    # print("dt is: ", dt)

                    #compute error
                    error = desired_distance - current_distance
                    error_sum = error_sum + error * dt
                    error_diff = (error - error_prev) / dt
                    error_prev = error

                    #compute output
                    output = Kp * error + Ki * error_sum + Kd * error_diff
                    
                    #print out the errors
                    # print("error is: ", error)
                    #print("error_sum is: ", error_sum)
                    # print("error_diff is: ", error_diff)
                    # print("output is: ", output)

                    # prevent the robot from getting stuck in deadlock
                    if error > 0.1:
                        left = 10
                        right = 10
                    else:
                        left = default_speed - output
                        right = default_speed + output
                    # print("left is: ", left)
                    # print("right is: ", right)
                    # print("*******************end**************************")
                    robot.set_vel(left, right)
            except Exception as e:
                log.write(f"An error occurred: {e}") 
                log.flush()

            # robot.set_vel(max(int(default_speed - output), 0), max(int(default_speed + output), 0))
           
            