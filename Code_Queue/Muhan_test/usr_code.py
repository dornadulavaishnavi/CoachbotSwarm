import time
import math
import random
import struct
import traceback


def get_theta(unit_vector):
    """
    Returns theta value from [-pi, pi] for a given vector.

    Args:
        unit_vector: A unit vectors array

    Returns:
        angle, in [-pi, pi]
    """
    x, y = unit_vector.x, unit_vector.y
    if y >= 0:
        return math.acos(x)
    else:
        return -math.acos(x)


def get_theta_diff(theta_target, theta_current):
    """
    Computes the minimum difference between two angles,
    negative result indicates left and positive result
    indicates right.

    Args:
        theta_target: Target angle to reach, in [-pi, pi]
        theta_current: Current angle, in [-pi, pi]

    Returns:
        angle difference, in [-pi, pi]
    """
    diff = theta_target - theta_current
    if math.fabs(diff) > math.pi:
        if theta_target > theta_current:
            # target > 0, current < 0
            return math.pi * 2 - theta_target + theta_current
        else:
            # target < 0, current > 0
            return theta_current - math.pi * 2 - theta_target
    else:
        return -diff


def get_pose_safe(robot):
    """
    Gets the pose info of the robot, until a result is returned
    """
    pose = None
    while not pose:
        pose = robot.get_pose()
    return Vector2D(pose[0], pose[1]), pose[2]


def clip(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value


class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x - other.x, self.y - other.y)
        elif isinstance(other, (int, float)):
            return Vector2D(self.x - other, self.y - other)
        else:
            raise TypeError("Unsupported operand type for subtraction")

    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x + other.x, self.y + other.y)
        elif isinstance(other, (int, float)):
            return Vector2D(self.x + other, self.y + other)
        else:
            raise TypeError("Unsupported operand type for addition")

    def __truediv__(self, value):
        return Vector2D(self.x / value, self.y / value)

    def __mul__(self, value):
        return Vector2D(self.x * value, self.y * value)

    def __repr__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"


class PositionEstimate:
    def __init__(self, name=""):
        self.last_position = None
        self.last_time = None
        self.speed_estimate = None
        self.name = name

    def update(self, new_position):
        new_time = time.time()
        if self.last_position is not None:
            delta_time = new_time - self.last_time
            self.speed_estimate = (new_position - self.last_position) / delta_time
        self.last_position = new_position
        self.last_time = new_time

    def get_position_estimate(self):
        if self.speed_estimate is not None:
            passed_time = time.time() - self.last_time
            return self.speed_estimate * passed_time + self.last_position
        else:
            return None

    def get_speed_estimate(self):
        return self.speed_estimate


def usr(robot):
    desired_distance = 0.3  # will vary from 0.3-0.5

    ################################
    # CONFIGURATIONS
    ################################
    robot_1_orbit_speed = 10
    robot_1_correction_max_speed = 10
    # maximum bound of absolute distance error when we apply the max correction speed
    # If a more stable trace is desired, use 0.1 (looser bound),
    # otherwise use 0.05 (tighter bound) to keep robot within desired distance
    robot_1_correction_distance_bound = 0.1
    # weight for mapping speed estimate to wheel speed
    robot_1_speed_estimate_weight = 100
    # weight for mapping difference between target speed direction and current direction
    # to wheel speed
    robot_1_wheel_diff_weight = 5

    ################################
    # STATES
    ################################
    robot.delay(3000)
    robot_0_position_estimate = PositionEstimate("robo0")
    change_timer = time.time()
    robot_id = robot.assigned_id
    try:
        while True:
            robot.delay()
            if robot_id == 0:
                position, direction = get_pose_safe(robot)
                robot.send_msg(struct.pack("ff", position.x, position.y))

                msgs = robot.recv_msg()
                if len(msgs) > 0:
                    robot_1_position = Vector2D(*struct.unpack("ff", msgs[0][:8]))
                    distance = (position - robot_1_position).length()

                    if distance > desired_distance:
                        # red
                        robot.set_led(100, 0, 0)
                    else:
                        # green
                        robot.set_led(0, 100, 0)

                if time.time() - change_timer > 3:
                    change_timer = time.time()
                    robot.set_vel(random.random() * 10, random.random() * 10)

            if robot_id == 1:
                msgs = robot.recv_msg()

                if len(msgs) > 0:
                    robot_0_position = Vector2D(*struct.unpack("ff", msgs[0][:8]))
                    # Update position estimate of robot 0 when receiving position information
                    robot_0_position_estimate.update(robot_0_position)

                position, direction = get_pose_safe(robot)

                # send current position information to robot 0
                robot.send_msg(struct.pack("ff", position.x, position.y))

                # update control every time we get current position
                # we cannot use position estimate on robot1 because the update
                # speed will be much higher, and in every update the speed of
                # robot 1 is updated, resulted in a high error
                robot_0_position = robot_0_position_estimate.get_position_estimate()
                if robot_0_position is not None:
                    # position difference vector from robot 1 to 0
                    position_diff_vec = robot_0_position - position

                    # unit vector of orbit speed term, always vertical to
                    # vector pointing from robot 1 to 0
                    orbit_speed_u_vec = Vector2D(
                        -position_diff_vec.y, position_diff_vec.x
                    )
                    orbit_speed_u_vec = orbit_speed_u_vec / orbit_speed_u_vec.length()

                    # unit vector of correction speed term, used for correcting
                    # distance error between robot 1 and 0
                    correction_speed_u_vec = (
                        position_diff_vec / position_diff_vec.length()
                    )

                    # positive correction_strength is attractive and negative is repulsive
                    correction_strength = clip(
                        (position_diff_vec.length() - desired_distance)
                        / robot_1_correction_distance_bound,
                        -1,
                        1,
                    )

                    # final speed vector is a mixture of orbit speed, correction speed,
                    # and adding the speed estimate of robot 0
                    speed_vec = (
                        orbit_speed_u_vec * robot_1_orbit_speed
                        + correction_speed_u_vec
                        * correction_strength
                        * robot_1_correction_max_speed
                        + robot_0_position_estimate.get_speed_estimate()
                        * robot_1_speed_estimate_weight
                    )

                    # move wheels
                    speed = speed_vec.length()
                    speed_direction = get_theta(speed_vec / speed)

                    # compute minimum theta difference between target speed direction
                    # and current direction
                    min_diff = get_theta_diff(speed_direction, direction)

                    # correct speed using a proportional controller
                    robot_1_wheel_speeds = [
                        speed + robot_1_wheel_diff_weight * min_diff,
                        speed - robot_1_wheel_diff_weight * min_diff,
                    ]

                    robot.set_vel(robot_1_wheel_speeds[0], robot_1_wheel_speeds[1])
    except Exception as e:
        print(traceback.format_exc())
        print(e)
