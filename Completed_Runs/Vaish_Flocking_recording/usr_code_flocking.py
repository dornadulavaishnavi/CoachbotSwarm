def usr(robot):
    robot.logger.info("Started Code")
    log = open("experiment_log", "wb")
    log.write("in user code")
    log.flush()
    
    # ME 409 
	# Lab 4
	# 11/29/2022
	# Written by Megan Black
	# This program demonstrates flocking of robots towards a goal location


    import struct
    import math
    import time

    # Define the goal location for the swarm to migrate to
    goalLoc = [0.0, 0.0]

    # Scaling parameters for each vector
    # Migration
    M = 0.5 # 0.75 # 1.0/2.0 
    # Separation
    S = 1.0 # 1.2
    # Cohesion
    C = 1.5 # 1.0
    #Alignment
    A = 1.0

    # Start robot movement on startup, creates non-zero velocity at beginning
    robot.set_vel(30, 30)

    count = 0

    robot.logger.info("got here")
    virt_id = robot.virtual_id()
    robot.logger.info("got id from function")
    robot.logger.info(virt_id)
    log.write(str(virt_id))
    
    while count<18:    
        log.write("\nlooping \n")
        log.flush()
        # Initialize dictionary of neighbors during first iteration
        try:
            a = Neighbors
        except:
            Neighbors = {}
                

        # Determine pose and send message
        
        # Get first pose
        pose_t = robot.get_pose()
        if pose_t:
            # First pose found
            pose1 = pose_t
            
            # Wait for 0.1 seconds while robot is moving
            currentTime = robot.get_clock()
            startTime = currentTime
            newTime = currentTime + 0.1
            while currentTime <= newTime:
                currentTime = robot.get_clock()

            # Get second pose
            pose_t2 = robot.get_pose()
            if pose_t2:
                # Second pose found
                pose2 = pose_t2

                # Calculate robot's velocity 
                xVel = ( pose2[0] - pose1[0] ) / (currentTime - startTime)
                yVel = ( pose2[1] - pose1[1] ) / (currentTime - startTime)
                #print('xVel = ', xVel, ' and yVel = ', yVel, ' for Robot ', robot.id)

                # Send message with current location, heading, velocity, and robot ID
                # [x location, y location, heading, x velocity, y velocity, robot ID]
                robot.send_msg(struct.pack('fffffi', pose2[0], pose2[1], pose2[2], xVel, yVel, robot.id))

           
        # Recieve messages from neighbors

        msgs = robot.recv_msg()
        ind = 0
        # Loop through message buffer
        while ind < len(msgs):
            # Unpack recieved message
            pose_rxed = struct.unpack('fffffi', msgs[ind][:24])

            # Update Neighbors with most recent position/velocity
            Neighbors[pose_rxed[5]] = pose_rxed[0:5]

            ind = ind + 1


        # Migration
        try:
            migration = [ goalLoc[0] - pose2[0], goalLoc[1] - pose2[1] ]

        except:
            print('Migration error')


        # Separation, Cohesion, Alignment
        try:
            # Set summing variables to 0 
            # Repulsion
            repul = [0, 0]
            # Center of Mass (cohesion)
            comX = 0
            comY = 0
            # Alignment
            alignX = 0
            alignY = 0

            # Loop thru each neighbor (nID) in the Neighbors dictionary
            for nID in Neighbors:

                # Repulsion summing
                # Distance between robot and the neighbor
                distRepul = math.sqrt( (pose2[0] - Neighbors[nID][0])**2 + (pose2[1] - Neighbors[nID][1])**2)
                # Calculate unit vector for repulsion away from neighbor
                nRepul = [(pose2[0] - Neighbors[nID][0])/distRepul**2,  (pose2[1] - Neighbors[nID][1])/distRepul**2 ]
                # Add to repulsion force
                repul[0] = repul[0] + nRepul[0]
                repul[1] = repul[1] + nRepul[1]

                # Cohesion summing
                # Add neighbor's position to the center of mass sum
                comX = comX + Neighbors[nID][0]
                comY = comY + Neighbors[nID][1]

                # Alignment
                # Add neighbor's velocity to the sum
                alignX = alignX + Neighbors[nID][3]
                alignY = alignY + Neighbors[nID][4]

                
            # Calculate unit vector for separation
            magRepul = math.sqrt( repul[0]**2 + repul[1]**2 )
            # Check if mag(repulsion vector) is non-zero
            if magRepul > 0:
                separation = [ repul[0]/magRepul, repul[1]/magRepul ]
            elif magRepul == 0:
                separation = repul
            

            # Add the location of the robot to the COM calculation and divid by the number of robots
            comX = ( comX + pose2[0] ) / ( len(Neighbors) + 1 )
            comY = ( comY + pose2[1] ) / ( len(Neighbors) + 1 )

            # Calculate vector towards COM
            cohesion = [ comX - pose2[0], comY - pose2[1] ]

            # Calculate unit vector for cohesion
            magC = math.sqrt( cohesion[0]**2 + cohesion[1]**2 )
            if magC > 0:
                cohesion = [ cohesion[0]/magC, cohesion[1]/magC]


            # Divide sum of velocities by number of neighbors to find average velocity
            alignX = alignX / len(Neighbors)
            alignY = alignY / len(Neighbors)

            # Calculate the unit vector for alignment
            magA = math.sqrt( alignX**2 + alignY**2 )
            alignment = [alignX, alignY]
            if magA > 0:
                alignment = [ alignment[0] / magA, alignment[1] / magA ]
 
        except:
            print('Separation/cohesion/alignment error')


        # Sum Vectors
        try:
            x = M*migration[0] + S*separation[0] + C*cohesion[0] + A*alignment[0]
            y = M*migration[1] + S*separation[1] + C*cohesion[1] + A*alignment[1]
        except:
            print('No vectors yet!')


        # Determine Heading
        try: 
            heading = math.atan2(y,x)
        except:
            print('No (x,y)')

        # Update Motion
        try:
            currentHeading = pose2[2]
            # Calculate how many radians to turn CW (right) and CCW (left)
            distR = math.fmod(abs(currentHeading - heading + 2*math.pi), 2*math.pi)
            distL = math.fmod(abs(2*math.pi - (currentHeading - heading)), 2*math.pi)

            # Define wheel speeds
            fastWheel = 30
            # Tune slower wheel between [15, 30] 
            # For turn angles greater than pi/2, the wheel speed is set to 15 to prevent sharp turning
            slowWheel = max(15, -30/math.pi * min(distL, distR) + 30)
            
            # Set left and right wheel velocities to turn in shortest direction
            # Turn CW (right)
            if distR <= distL:
                velR = slowWheel
                velL = fastWheel
            # Turn CCW (left)
            elif distL < distR:
                velR = fastWheel
                velL = slowWheel

            # Set robot's wheel velocities
            robot.set_vel(velL, velR)

            # Move for a short amount of time (wait 0.4 seconds)
            # This allows for faster flocking behavior
            currentTime = robot.get_clock()
            newTime = currentTime + 0.4
            while currentTime < newTime:
                currentTime = robot.get_clock()
            
        except:
            print('No heading')


        # Print the current and desired headings
        # For debugging purposes
        try:
            print('Heading = ', pose2[2], ', Desired Heading = ', heading, ' for Robot ', robot.id)

        except:
            print('This did not work :(')

        count += 1
        robot.delay(500)
    log.close()








