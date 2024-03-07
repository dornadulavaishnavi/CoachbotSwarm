import random

def usr(robot):
    robot.delay(3000)
    T = random.randint(1000,1300)
    # print(T)
    id = robot.id


    # pose = robot.get_pose()
    # print(id, pose)
    if id == 5:
        robot.delay(1000)
    while True:
        # robot.start_new_loop()
        robot.delay()
        
        if id == 5:
            robot.set_led(0,100,0)
            # print("id: ", id, "turn on")
            robot.delay(200)
            curr_time = robot.get_clock()
            curr_time = round(curr_time, 2)
            msg = "time:"+ str(curr_time)+';'
            robot.send_msg("led:(0,100,0);delay:800;"+msg)
            robot.set_led(0,0,0)
            # print("id: ", id, "turn off")
            robot.delay(800)
            

        else:
            val = robot.recv_msg(clear=True)
            
            if len(val)>0:
                # print(val)
                if len(val[0]) > 1:
                    txt = val[0]
                    # print(val)
                    sub_str = 'delay'
                    res = [i for i in range(len(txt)) if txt.startswith(sub_str, i)]

                    # print(txt)
                    # Get the delay
                    ind = res[-1]
                    txt = txt[ind:]
                    # msg_time = "time:"+ str(curr_time)+';'
                    # r, g, b = (0, 100, 0)
                    # msg = "led:("+ str(r)+","+str(g)+","+str(b)+");" + txt
                    # robot.send_msg(msg)
                    # print(ind)
                    
                    
                    # print(txt)
                    ind_delim = txt.index(";")
                    # print(ind_delim)
                    delay_val = txt[6:ind_delim]
                    # print("Delay_val:",delay_val)
                    delay_val = float(delay_val)
                    
                    # Get the time 
                    ind = txt.index("time")
                    ind_delim = ind +1 + txt[ind+1:].index(";")
                    time_val = txt[ind+5:ind_delim]
                    time_val = float(time_val)
                    curr_time = robot.get_clock()
                    
                    next_time = time_val + delay_val/1000
                    # print("Next time:",next_time)
                    T = delay_val
                    del_Sleep = next_time - curr_time - 0.05
                    # print("Del Sleep:",del_Sleep)
                    if del_Sleep> 0:
                        robot.delay(int(del_Sleep*1000))
                    
            r, g, b = (0, 100, 0)
            robot.set_led(r,g,b)
            robot.delay(200)
            T = int(T)
            curr_time = robot.get_clock()
            msg_time = "time:"+ str(curr_time)+';'
            msg = "led:("+ str(r)+","+str(g)+","+str(b)+");" + "delay:"+ str(T)+";" + msg_time
            # robot.send_msg(msg)
            robot.set_led(0,0,0)
            robot.delay(T)
        