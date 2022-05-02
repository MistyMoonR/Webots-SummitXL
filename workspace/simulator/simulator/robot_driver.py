import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, atan2
from rosgraph_msgs.msg import Clock


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.123
LX = 0.2045 #lateral distance from robot's COM to wheel [m].
LY =0.2225 #longitudinal distance from robot's COM to wheel [m].
target_speed = {'x': 1.0, 'y': 1.0, 'z': 1.0}


class main:
    def init(self, webots_node, properties):

        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.fl_motor = self.__robot.getDevice('front_left_wheel_joint')
        self.fr_motor = self.__robot.getDevice('front_right_wheel_joint')
        self.bl_motor = self.__robot.getDevice('back_left_wheel_joint')
        self.br_motor = self.__robot.getDevice('back_right_wheel_joint')
        self.fl_motor.setPosition(float('inf'))
        self.fl_motor.setVelocity(0)
        self.fr_motor.setPosition(float('inf'))
        self.fr_motor.setVelocity(0)
        self.bl_motor.setPosition(float('inf'))
        self.bl_motor.setVelocity(0)
        self.br_motor.setPosition(float('inf'))
        self.br_motor.setVelocity(0)

        rclpy.init(args=None)

        self.__node = rclpy.create_node('robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
       self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        vx = self.__target_twist.linear.x
        vy = self.__target_twist.linear.y
        wz = self.__target_twist.angular.z

        fl, fr, bl, br = mecanumControl(vx, vy, wz)

        self.bl_motor.setVelocity(fl)
        self.fl_motor.setVelocity(fr)
        self.br_motor.setVelocity(bl)
        self.fr_motor.setVelocity(br)


def mecanumControl(vx, vy, wz):
    fl = 1 / WHEEL_RADIUS * (vx - vy - ((LY + LY) * wz) )
    fr = 1 / WHEEL_RADIUS * (vx + vy - ((LY + LY) * wz) )
    bl = 1 / WHEEL_RADIUS * (vx + vy + ((LY + LY) * wz) )
    br = 1 / WHEEL_RADIUS * (vx - vy + ((LY + LY) * wz) )
    return fl, fr, bl, br

