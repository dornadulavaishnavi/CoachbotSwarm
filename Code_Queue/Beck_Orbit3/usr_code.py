def usr(robot):
    import struct #enables struct use
    import math #enables math use
    desired_distance = .4 #will vary from 0.3-0.5, feel free to change it, Professor!

    previous_deviation = 0 #stored deviation from previous iteration
    deviation = 0 #deviation between target distance and actual distance
    
    robot.delay(3000)

    log = open("experiment_log", "w")
    
    while True:
        robot.delay()
        if (robot.virtual_id() == 0):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])

                pose_t = robot.get_pose() # get position to store inside logic

                distx = pose_t[0] - pose_rxed[0] # calculate x displacement
                disty = pose_t[1] - pose_rxed[1] # calculate y displacement
                dist = math.sqrt((distx * distx) + (disty * disty)) # calculate distance

                log.write("Distance: ", dist)

                if dist > desired_distance:
                    #blink led red if message is received and too far 
                    robot.set_led(100,0,0)
                    robot.delay(10)
                    robot.set_led(0,0,0)
                else:
                    #blink led green if message is received and too close
                    robot.set_led(0,100,0)
                    robot.delay(10)
                    robot.set_led(0,0,0)

            pose_t = robot.get_pose() # get position to store outside logic

            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.virtual_id()))  # send pose x,y in message
         
         
        if (robot.virtual_id() == 1):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12]) #receive message

                pose_t = robot.get_pose() #get robot position to store in logic

                distx = pose_t[0] - pose_rxed[0] #calculate x displacement
                disty = pose_t[1] - pose_rxed[1] #calculate y displacement
                dist = math.sqrt((distx * distx) + (disty * disty)) #calculate distance

                deviation = dist - desired_distance #calculate deviation

                if dist > desired_distance:
                    #blink led red if message is received and too far
                    robot.set_led(100,0,0)
                    robot.delay(10)
                    robot.set_led(0,0,0)
                else:
                    #blink led green if message is received and too close
                    robot.set_led(0,100,0)
                    robot.delay(10)
                    robot.set_led(0,0,0)


            pose_t = robot.get_pose() #get position for storage outside of logic

            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.virtual_id()))  # send pose x,y in message

            #determined formula: right wheel power = 10 - (20 / (25 * desired radius + 1))    

            right_den = (25 * desired_distance) + 1 #calculate denominator of formula

            right_power = 10 - (20 / right_den) #calculate power of right wheel

            if deviation > 0:
                if deviation > previous_deviation:
                    right_power -= 25 * deviation #move in tighter circle
                else:
                    right_power -= 2 * deviation #move in marginally tighter circle
            elif deviation < 0:
                if deviation < previous_deviation:
                    right_power = 10 #go straight
                else:
                    right_power += 2 * deviation #move in marginally wider circle

            previous_deviation = deviation #store deviation for delta deviation calculation

            robot.set_vel(10, right_power) #move wheels
    log.close()
    return