import struct
import numpy as np


def usr(robot):
    robot.delay(
        3000
    )  # ensures that the camera and all other peripherals are up and running before your code begins
    # any set up variables or code before looping can go here
    log = open("experiment_log", "w")
    log.write("Experiment start\n")
    log.flush()

    desired_distance = 0.3  # will vary from 0.3-0.5

    ################################
    # CONFIGURATIONS
    ################################
    robot_1_orbit_speed = 10
    robot_1_kP = 45
    robot_1_kI = 0
    robot_1_kD = 100

    ################################
    # STATES
    ################################
    robot_0_position = None
    robot_1_position = None
    robot_id = robot.virtual_id()
    distance_error = 0
    distance_error_diff = 0
    distance_error_sum = 0
    while True:
        robot.delay()
        if robot_id == 0:
            pose_0 = robot.get_pose()
            if pose_0:
                robot_0_position = np.array(pose_0[:2])
                robot.send_msg(struct.pack("ff", pose_0[0], pose_0[1]))

            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_1 = np.array(struct.unpack("ff", msgs[0][:8]))
                if robot_0_position is not None:
                    robot_1_position = np.array(pose_1)
                    distance = np.linalg.norm(robot_0_position - robot_1_position)
                    log.write("robot 0 " + str(distance) + "\n")
                    log.flush()
                    if distance > desired_distance:
                        robot.set_led(100, 0, 0)
                    else:
                        robot.set_led(0, 100, 0)
        if robot_id == 1:
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_0 = struct.unpack("ff", msgs[0][:8])
                robot_0_position = np.array(pose_0)
            pose_1 = robot.get_pose()
            if pose_1:
                # send current position information to robot 0
                robot.send_msg(struct.pack("ff", pose_1[0], pose_1[1]))
                robot_1_position = np.array(pose_1[:2])

                if robot_0_position is not None:
                    # position difference vector from robot 1 to 0
                    position_diff_vec = robot_0_position - robot_1_position
                    distance = np.linalg.norm(position_diff_vec)

                    log.write("robot 1 " + str(distance) + "\n")
                    log.flush()
                    if distance > desired_distance:
                        robot.set_led(100, 0, 0)
                    else:
                        robot.set_led(0, 100, 0)

                    last_distance_error = distance_error
                    distance_error = distance - desired_distance
                    distance_error_diff = distance_error - last_distance_error
                    distance_error_sum += distance_error

                    # correct speed using a PID controller
                    turn = (
                        robot_1_kP * distance_error
                        + robot_1_kI * distance_error_sum
                        + robot_1_kD * distance_error_diff
                    )
                    robot_1_wheel_speeds = np.array(
                        [
                            robot_1_orbit_speed + turn,
                            robot_1_orbit_speed - turn,
                        ]
                    )

                    robot.set_vel(robot_1_wheel_speeds[0], robot_1_wheel_speeds[1])

