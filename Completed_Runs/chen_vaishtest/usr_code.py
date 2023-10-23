import math
import struct
import pdb

#chen

desired_distance = 0.45 # between 0.3 and 0.5

# parameters for tuning the P controllers
gain_orientation = 2
gain_linear = 10

def msg2dict(msg):
    # Convert message string into dict
    pairs = msg.split(';')
    result_dict = {}
    for pair in pairs:
        key, value = pair.split(':')
        key = key.strip()
        value = value.strip()
        if ',' in value:
            value = [float(x.strip()) for x in value[1:-1].split(',')]
        else:
            try:
                value = float(value)
            except ValueError:
                value = value
        result_dict[key] = value
        return result_dict

def calculate_distance(pose1, pose2):
    # Calculate the distance between two robots
    x1, y1, orientation1 = pose1
    x2, y2, orientation2 = pose2
    distance_xy = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance_xy


def usr(robot):
    robot.delay(3000) # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "wb")
    
    log.write("an example write string\n")
    log.flush()
    id = robot.virtual_id()

    while True:
        robot.delay()
        # your looping code here
        if id == 0:
            pose = robot.get_pose()
            msg = "pose:"+ str(pose)+';'
            robot.send_msg(msg)
            val = robot.recv_msg(clear=True)
            if val != []:
                info = msg2dict(val[0])
                distance = info["distance"]
                distance_error = float(distance) - desired_distance
                # Set the robot color according to the distance errors as required
                if distance_error > 0:
                    robot.set_led(100,0,0)
                else:
                    robot.set_led(0,100,0)
                print("[ROBOT 0] Distance: ", distance)

        elif id == 1:
            val = robot.recv_msg(clear=True)
            # Move when receiving messages
            if val != [] and val != [""]:
                info = msg2dict(val[0])
                robot0pose = info["pose"]
                mypose = robot.get_pose()
                # calculate the distance between robot 1 and robot 0
                distance = calculate_distance(robot0pose, mypose)
                # Send this distance information to robot 0
                msg = "distance:"+ str(distance)+';'
                robot.send_msg(msg)
                # Calculate the distance error
                distance_error = distance - desired_distance
                # Set the robot color according to the distance errors as required
                if distance_error > 0:
                    robot.set_led(100,0,0)
                else:
                    robot.set_led(0,100,0)
                # Robot 1 should be perpendicular to the line between robot 1 and robot 0
                desired_angle = math.atan2(robot0pose[1] - mypose[1], robot0pose[0] - mypose[0]) + math.pi/2
                # Calculate the orientation error
                orientation_error = desired_angle - mypose[2]
                orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
                # Calculate the velocities for two wheels based on the distance error and orientation error
                linear_vel = gain_linear * (distance_error + 0.05)
                angular_vel = gain_orientation * orientation_error
                left_wheel_vel = linear_vel-angular_vel + 5
                right_wheel_vel = -linear_vel+angular_vel + 5
                # Actuate the robot
                robot.set_vel(left_wheel_vel,right_wheel_vel) 

    log.close()
    return
