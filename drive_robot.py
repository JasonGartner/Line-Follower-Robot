import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

gx = 4.0  # goal x pos
gy = 5.0  # goal y pos
point_x = 0.0  # temp var
point_y = 0.0  # temp var


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')
        self._goal_x = gx
        self._goal_y = gy
        self._goal_t = 0.0

        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

    def _listener_callback(self, msg, vel_gain=5.0, max_vel=1.0, max_pos_err=0.1):
        pose = msg.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw

        Arr_o1 = []  # circumference values for circle 1
        Arr_o2 = []  # circumference values for circle 2
        x_o1 = 3  # x center of circle 1
        y_o1 = 4  # y center of circle 1
        x_o2 = 4  # x center of circle 2
        y_o2 = 1  # y center of circle 1
        r_rob = 0.05  # radius of robot
        r_o1 = 0.2  # radius of circle 1
        r_o2 = 0.3  # radius of circle 2
        tangent_slope = 0
        b = 0
        tmp_r1 = r_rob + r_o1 + 0.45  # radius of outer circle for robot to follow for circle 1
        tmp_r2 = r_rob + r_o2 + 0.45  # radius of outer circle for robot to follow for circle 2

        # o1 calculate all points on circumference of outer circle
        for i in range(360):
            rx = tmp_r1 * math.cos(i * math.pi / 180)
            ry = tmp_r1 * math.sin(i * math.pi / 180)
            tmp_pos1 = [round(rx + x_o1, 3), round(ry + y_o1, 3)]
            Arr_o1.append(tmp_pos1)
        print(Arr_o1)
        # o2 calculate all points on circumference of outer circle
        for i in range(360):
            rx = tmp_r2 * math.cos(i * math.pi / 180)
            ry = tmp_r2 * math.sin(i * math.pi / 180)
            tmp_pos2 = [round(rx + x_o2, 3), round(ry + y_o2, 3)]
            Arr_o2.append(tmp_pos2)

        circle_diff_1 = abs(tmp_r1 - math.sqrt((y_o1 - cur_y) * (y_o1 - cur_y) + (x_o1 - cur_x) * (
                    x_o1 - cur_x)))  # checks to see when the robot reaches the circle
        circle_diff_2 = abs(tmp_r1 - math.sqrt((y_o2 - cur_y) * (y_o2 - cur_y) + (x_o2 - cur_x) * (
                    x_o2 - cur_x)))  # checks to see when the robot reaches the circ

        if circle_diff_1 < (0.05):  # move around circle 1
            for i in Arr_o1:
                if abs(i[0] - cur_x) < 0.05 and abs(
                        i[1] - cur_y) < 0.05:  # gets current position in array of points
                    index = Arr_o1.index(i)  # gets the inxex of our current location from the temp arry
                    # determins where the next position where we need to move the robot on the perimeter of the circle path
                    if index + 10 > len(Arr_o1) - 1: # move to next point around circle
                        point_x = Arr_o1[(index + 20) % 360][0]
                        point_y = Arr_o1[(index + 20) % 360][1]
                    else:
                        point_x = Arr_o1[index + 20][0]
                        point_y = Arr_o1[index + 20][1]

                    # calculate tangent line
                    slope = (y_o1 - cur_y) / (x_o1 - cur_x)
                    tangent_slope = -(1 / slope)
                    b = cur_y - cur_x * slope

            if abs(gy - tangent_slope * gx + b) < 1.5:  # we exit
                # move to goal
                self._goal_x = gx
                self._goal_y = gy
            else:
                # move around the circular path
                self._goal_x = point_x
                self._goal_y = point_y

        if circle_diff_2 < (0.05):  # move around circle 2
            for i in Arr_o2:
                if abs(i[0] - cur_x) < 0.05 and abs(
                        i[1] - cur_y) < 0.05:  # gets current position in array of points
                    index = Arr_o2.index(i)  # gets the inxex of our current location from the temp arry
                    # determins where the next position where we need to move the robot on the perimeter of the circle path
                    if index + 10 > len(Arr_o2) - 1: # move to next point around circle
                        point_x = Arr_o2[(index + 20) % 360][0]
                        point_y = Arr_o2[(index + 20) % 360][1]
                    else:
                        point_x = Arr_o2[index + 20][0]
                        point_y = Arr_o2[index + 20][1]

                    # calculate tangent line
                    slope = (y_o2 - cur_y) / (x_o2 - cur_x)
                    tangent_slope = -(1 / slope)
                    b = cur_y - cur_x * slope

            if abs(gy - tangent_slope * gx + b) < 1.5:  # we exit
                # move to goal
                self._goal_x = gx
                self._goal_y = gy

            else:
                # move around the circular path
                self._goal_x = point_x
                self._goal_y = point_y

        # dist = distance (of both x and y components)
        # x_diff = x distance component
        # y_diff = y distance component
        # x = x velocity
        # y = y velocity

        x_diff = self._goal_x - cur_x  # x distance from goal
        y_diff = self._goal_y - cur_y  # y distance from goal
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        twist = Twist()
        if dist > max_pos_err:
            x = x_diff * vel_gain  # x velocity (set direction)
            y = y_diff * vel_gain  # y velocity (set direction)
            vel = math.sqrt(x * x + y * y)  # velocity magnatude
            if x_diff < 0:  # sets angle baised off what quadrent we are in (flipped since x_diff points to origin and is opposite of current x position, if x=-3 then x_diff = 3)
                angle = math.pi + math.atan(y / x)  # angle to goal
            else:
                angle = math.atan(y / x)  # angle to goal
            if vel > max_vel:  # if the velocity is greater than out max_vel then we need to cap it to 40
                x = max_vel * math.cos(angle)
                y = max_vel * math.sin(angle)

            # insert if statements here

            twist.linear.x = x * math.cos(cur_t) + y * math.sin(cur_t)
            twist.linear.y = -x * math.sin(cur_t) + y * math.cos(cur_t)
            self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t}) goal ({self._goal_x},{self._goal_y},{self._goal_t})")
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
