import math
import struct
import numpy as np

def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string\n")
    log.flush()

    
    desired_distance = 0.45 # will vary from 0.3-0.5
    
    # Init storage
    r0x = 0.0
    r0y = 0.0
    x0 = 0.0
    y0 = 0.0
    
    # Init err storage
    total_err = 0
    d = None

    while True:
        robot.delay()
        if (robot.id == 0):
            log.write("Is robot 0\n")
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])

                # Grab robot 1 coordinates
                x1 = pose_rxed[0]
                y1 = pose_rxed[1]

                # Calculate distance, print, and color robot 
                robot_dist = np.sqrt((x0-x1)**2+(y0-y1)**2)
                print("The distance between the two robots is: "+str(robot_dist))
                if(robot_dist > desired_distance): robot.set_led(100,0,0)
                else: robot.set_led(0,100,0)

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                #print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message
                # Grab robot 0 coordinates
                x0 = pose_t[0]
                y0 = pose_t[1]    

            robot.set_vel(0, 0)

        if (robot.id == 1):
            log.write("Is robot 1\n")
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])

                # Grab r0 positions
                # If some more advanced control was required, would want to 
                # keep a buffer to estimate velocity. Uneeded for now.
                r0x = pose_rxed[0]
                r0y = pose_rxed[1]

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                #print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message

                # Acquire Pose
                r1x = pose_t[0]
                r1y = pose_t[1]
            
            # Calculate distance in x, y
            del_x = r0x-r1x
            del_y = r0y-r1y
            
            # Determine distance between two robots 
            # Include some logic for starting error measurement
            if(d is None): 
                d = np.sqrt((del_x)**2 + (del_y)**2)
                last_d = d
            else:
                last_d = d
                d = np.sqrt((del_x)**2 + (del_y)**2)
                
            # Set robot to proper color 
            # Green = too close or exact
            # Red = too far
            if(d > desired_distance): robot.set_led(100,0,0)
            else: robot.set_led(0,100,0)

            # Calculate error statistics 
            err = d - desired_distance
            diff_err = d-last_d
            total_err += err
            total_err = min(max(total_err,-100),100) # Arbitrary limits

            # kP needs to have sufficent power to correct and to follow a 
            # moving target
            kP = 45
            
            # Main purpose of kD is to damp the oscillation that comes with 
            # Turning in and out of the zone 
            kD = 100
            
            # Determined that kI was not needed and also adding kI can be scary
            # for longer term sims or when the robot_0 is moving 
            # Left in control in case of tuning later. 
            kI = 0.0
            
          
            # The wheels always move forward but their relative speeds are 
            # affected by the measured error. There is a slight differnece 
            # in their 0 error speeds to force a CW rotaiton
            
            # If the robot is too far (err > 0), speed up the outside wheel 
            # and slow inside wheel to turn in a closer circle. 
            
            # If robot is too close (err < 0), slow the outside wheel and 
            # speed up the inside wheel to turn in a wide circle
            
            # This will only for for clockwise orbiting.
            whl_1_spd = 11.125 + kP * (err) + kD * (diff_err) + kI*total_err
            whl_2_spd = 8.875 - kP * (err) - kD * (diff_err) - kI*total_err
                
            #Set wheel velocities left=outside, right=inside
            robot.set_vel(whl_1_spd, whl_2_spd)

        log.write("loop\n")
            
            
        # your looping code here

    log.close()
    return