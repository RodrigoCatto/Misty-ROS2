import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist

import mistyPy
import time
from numpy import clip

class MistyLowLevel(Node):

    def __init__(self):
        super().__init__('misty_low_level')
        print("Low level node created")

        # Load parameters
        my_parameter_descriptor = ParameterDescriptor(description='Misty IP Address')
        self.declare_parameter('ip_address', '0.0.0.0', my_parameter_descriptor)
        self.declare_parameter('robot_name', 'misty_0', my_parameter_descriptor)

        ip_adrress = self.get_parameter('ip_address').get_parameter_value().string_value
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.get_logger().info("{} at IP:{}".format(robot_name, ip_adrress))

        self.misty = mistyPy.Robot(ip_adrress) #TODO error handling
        self.misty.stop() # Make sure that Misty is halted

        self.vel_x = 0
        self.ang_z = 0

        self.misty_max_linear_speed = 0.7 # m/s or 2.5km/h
        self.misty_max_ang_speed = 6 # rad/s

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_subscriber

    def cmd_vel_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.vel_x = msg.linear.x
        self.ang_z = msg.angular.z
        self.set_misty_speed()

    def set_misty_speed(self):
        # Convert m/s to misty -100 to 100 range
        misty_vel_x = int(100*self.vel_x/self.misty_max_linear_speed)
        misty_angular_z = int(100*self.ang_z/self.misty_max_ang_speed)
        
        # Clips the speeds to a max of 100 and a min of -100
        misty_vel_x = int(clip(misty_vel_x, -100, 100))
        misty_angular_z = int(clip(misty_angular_z, -100, 100))

        if -10 > misty_vel_x < 10 and -10 > misty_angular_z < 10:  
            self.get_logger().info("Halt")
            self.misty.stop()
        else:
            self.get_logger().info("Linear Speed: {}  Angular Speed {}".format(misty_vel_x, misty_angular_z))
            self.misty.drive(misty_vel_x, misty_angular_z)


def main(args=None):
    rclpy.init(args=args)

    misty_low_level_node = MistyLowLevel()

    rclpy.spin(misty_low_level_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    misty_low_level_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




