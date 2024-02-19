import struct,math
def usr(robot):
    robot.delay(3000)
    mypos0 = robot.get_pose() #get position
    robot.send_msg(struct.pack('fff'),mypos0[0],mypos0[1],mypos0[2])
    youpos0 = struct.unpack('fff',msg[0][:4])
    mypos_final = youpos0.copy() #setting up where it needs to go

    nspin = 0
   
    #changing color
    robot.set_led(100,0,0)
    robot.delay(10000)
    robot.set_led(0,0,0)
    
    #swapping places with another robot
    mypos = robot.get_pose() #get position
    robot.send_msg(struct.pack('fff'),mypos[0],mypos[1],mypos[2])
    msg = robot.recv_msg()
    youpos = struct.unpack('fff',msg[0][:4])
    ourdist = math.sqrt(((mypos[0]-youpos[0]) ** 2)+(mypos[1]-youpos[1])**2)
    
    while True:
        robot.delay()
        # check distance of the two robots
        mypos = robot.get_pose() #get position
        robot.send_msg(struct.pack('fff'),mypos[0],mypos[1],mypos[2])
        msg = robot.recv_msg()
        youpos = struct.unpack('fff',msg[0][:4])
        ourdist = math.sqrt(((mypos[0]-youpos[0]) ** 2)+(mypos[1]-youpos[1])**2)
        
        robot.set_vel(10,10)
        
        if (ourdist < 0.2): #will move forward until too close
            robot.set_vel(0,0) #stop, rotate, then continue 
            robot.delay(200)
            angdiff = abs(abs(mypos[2])-abs(youpos[2]))
            
            while angdiff < 90.0: #compare new angle to make sure spin enough
                robot.delay()
                robot.set_vel(1,-1)
                robot.delay(2000)
                robot.set_vel(0,0)
                
                mypos = robot.get_pos()
                robot.send_msg(struct.pack('fff'),mypos[0],mypos[1],mypos[2])
                msg = robot.recv_msg()
                youpos = struct.unpack('fff',msg[0][:4])
                angdiff = abs(abs(mypos[2])-abs(youpos[2]))
                
                nspin = nspin+1
            x0 = mypos[0]
            robot.delay()
            robot.set_vel(10,10)
            robot.delay(4000)
            robot.set_vel(1,-1)
            robot.delay(2000*nspin)
            robot.set_vel(0,0)

            mypos = robot.get_pose() #get position again
            x1 = mypos[0]
            x_dist = abs(x0-x1) #distance need to re-turn at later
            robot.send_msg(struct.pack('fff'),mypos[0],mypos[1],mypos[2])
            msg = robot.recv_msg()
            youpos = struct.unpack('fff',msg[0][:4])
            
            ourdist = math.sqrt(((mypos[0]-youpos[0]) ** 2)+(mypos[1]-youpos[1])**2)
        
        robot.set_vel(10,10)

        mypos = robot.get_pose()
        if (abs(mypos[0]) > (abs(mypos_final[0])-x_dist)):
            robot.set_vel(1,-1)
            robot.delay(2000*nspin)
            robot.set_vel(10,10)
            robot.delay(4000)
            
            robot.set_vel(0,0)

            mypos = robot.get_pos() #rotating to final orientation
            while mypos[2] != mypos_final[2]:
                robot.delay()
                robot.set_vel(-1,1)
                robot.get_pos()
            robot.set_vel(0,0)
            robot.set_led(0,100,0)
    return






    
    
    