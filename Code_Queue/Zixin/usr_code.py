import math
import struct
import numpy as np

def usr(robot):
    robot.delay(3000)
    log = open("experiment_log", "wb")
    log.write("an example write string\n")
    log.flush()

    alpha = math.pi / 12

    while True:
        robot.delay()

        msgs = robot.recv_msg()

        pose = robot.get_pose()
        pos_j = np.array([pose[0], pose[1]])
        theta_j = pose[2]
        if pose:
            log.write(str(theta_j))
            log.flush()

            robot.send_msg(struct.pack("ffi", pos_j[0], pos_j[1], robot.virtual_id()))        
        
        id_count = np.zeros(10)

        seen_fish = False
        if len(msgs) > 0:
            for i in range(len(msgs)):
                if seen_fish == False:
                    recv_msg = struct.unpack("ffi", msgs[i])
                    if id_count[recv_msg[2]] == 0:
                        id_count[recv_msg[2]] = 1
                        pos_i = np.array([recv_msg[0], recv_msg[1]])
                        vec_ji = pos_i - pos_j
                        theta_ji = math.atan2(vec_ji[1], vec_ji[0])
                        if abs(theta_ji - theta_j) < alpha:
                            seen_fish = True
        
        if seen_fish == False:
            robot.set_vel(15, 10)
        else:
            robot.set_vel(10, 15)
