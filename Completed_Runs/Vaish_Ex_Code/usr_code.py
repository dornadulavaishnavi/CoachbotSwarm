import math
import struct

def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    log = open("experiment_log", "wb") # open the logger file
    
    # any variables to initialize or code before looping can go here
    num_loops = 5
    msg_count = 0
    strings_received = {}

    log.write("The example code has begun\n")
    log.flush() # remember to always call the flush() command after writing to the file

    while True:
        robot.delay()
        # your looping code here

        log.write("After delay\n")
        log.flush()

        curr_pose = robot.get_pose()    # get the current position of the robot
        while not curr_pose:    # in case the robot is unable to localize, wait until it can
            curr_pose = robot.get_pose()    # poses are received in a list of [x,y,theta]

        log.write("Got the pose\n")
        log.flush()

        # generate and send a message to all the other robots
        robot.send_msg(struct.pack('ifff', virtual_id(), curr_pose[0], curr_pose[1], curr_pose[2]))
        
        # print message to the log file
        debug_string = "Robot " + str(robot.virtual_id()) + " is facing " + str(curr_theta) + "\n"
        log.write(debug_string)
        log.flush()

        if robot.virtual_id()%2 == 0: # if my robot's assigned id is even
            robot.set_led(100,0,0)   # set the LED to red
            for i in range(num_loops):  # turn in a circle for 1 second (5 loops)
                robot.set_vel(45,-45)
                robot.delay(200)
        else: # for odd robots
            robot.set_led(100,0,100)    # set the LED to purple (half red and half blue)
            for i in range(num_loops):  # turn in the other direction for 1 second (5 loops)
                robot.set_vel(-45,45)
                robot.delay(200)

        log.write("Robot finished turning\n")
        log.flush()

        rec_str = robot.recv_msg()  # grab any messages sent
        if len(rec_str) > 0:    # if any messages have been received
            while i < (len(str)):
                # save each messaged received into a dictionary
                strings_received[i] = struct.unpack('ifff', rec_str[i][:16])
                msg_count += 1
            log.write("Robot has received messages\n")
            log.flush()

        if msg_count >= 10: # if the robot has received 10 or more messages, return
            log.write(str(strings_received))
            log.flush()
            log.close() # close the logger file
            return