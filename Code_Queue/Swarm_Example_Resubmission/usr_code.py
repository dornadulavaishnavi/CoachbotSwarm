import math
import struct

def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string\n")
    log.flush()

    while True:
        robot.delay()
        # your looping code here

        # set the robot's LED to blue
        robot.set_led(0,0,100)

        # first get our robot's current position
        curr_pose = robot.get_pose()
        while not curr_pose: # ensures that we get the pose, robot will return False if the position hasn't been updated
            curr_pose = robot.get_pose()
        curr_x = curr_pose[0]
        curr_y = curr_pose[1]
        curr_theta = curr_pose[2] 
        log.write("My current pose is " + str(curr_pose) + "\n")
        log.flush()

        # if robot is in the positive x area, turn to the left
        if curr_x >= 0:
            curr_time = robot.get_clock()
            while (robot.get_clock() - curr_time) < 5:
                robot.set_vel(-25,25)
            log.write("Turning left!\n")
            log.flush()
        
        # if robot is in negative x area, turn to the right
        if curr_x < 0:
            curr_time = robot.get_clock()
            while (robot.get_clock() - curr_time) < 5:
                robot.set_vel(25,-25)
            
            log.write("Turning Right!\n")
            log.flush()
        
        
        robot.set_vel(0,0)
        # send position information
        # update the position
        curr_pose = robot.get_pose()
        while not curr_pose: # ensures that we get the pose, robot will return False if the position hasn't been updated
            curr_pose = robot.get_pose()
        curr_x = curr_pose[0]
        curr_y = curr_pose[1]
        curr_theta = curr_pose[2] 

        # delay between sending and receiving messages
        robot.delay()

        # send the message with the ID of the robot and current position/orientation
        robot.send_msg(struct.pack('ifff', robot.virtual_id(), curr_x, curr_y, curr_theta))

        # if we received more than 5 messages, set LED to red
        msgs = robot.recv_msg() # msgs is a 2D list with the message item index in the first index and the message in the other direction
        num_msgs = 0
        unpacked_msgs = {}
        for index in range(len(msgs)):
            indv_msg = struct.unpack('ifff', msgs[index][:16]) # will return a list with those 4 items in it
            unpacked_msgs[indv_msg[0]] = [indv_msg[1], indv_msg[2], indv_msg[3]] # saving the messages in a dictionary as, {ID: [x,y,theta]}
            num_msgs += 1

        log.write("I received " + str(num_msgs) + " messages\n")
        log.flush()
        
        if num_msgs > 5:
            robot.set_led(100,0,0)

        robot.delay(3000)

        # if robot is in the positive y area, turn to the left
        if curr_y >= 0:
            curr_time = robot.get_clock()
            while (robot.get_clock() - curr_time) < 5:
                robot.set_vel(-25,25)
            log.write("Turning Left!\n")
            log.flush()

        # if robot is in negative y area, turn to the right
        if curr_y < 0:
            curr_time = robot.get_clock()
            while (robot.get_clock() - curr_time) < 5:
                robot.set_vel(25,-25)

            log.write("Turning Right!\n")
            log.flush()

        # wait for 5 seconds
        robot.delay(5000)

        # stop robot motion
        robot.set_vel(0,0)

        # if robot id is a modulus of 2, change LED color to green
        if robot.virtual_id()%2 == 0:
            robot.set_led(0,100,0)
        
        log.write("My ID is " + str(robot.virtual_id()) + "\n")
        log.flush()
        
        # wait 3 seconds
        robot.delay(3000)

        log.close()
        return