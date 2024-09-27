import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import mistyPy
import time
from numpy import clip

class MistyLowLevel(Node):

    def __init__(self):
        super().__init__('misty_low_level')
        print("Low level node created")
        self.misty = mistyPy.Robot("192.168.1.3") #TODO make the IP as a parameter or class input
        self.misty.stop() # Make sure that Misty is stopped

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
        #Convert m/s to misty -100 to 100 range
        misty_vel_x = int(100*self.vel_x/self.misty_max_linear_speed)
        misty_angular_z = int(100*self.ang_z/self.misty_max_ang_speed)
        
        misty_vel_x = int(clip(misty_vel_x, -100, 100))
        misty_angular_z = int(clip(misty_angular_z, -100, 100))

        if -10 > misty_vel_x < 10 and -10 > misty_angular_z < 10:
            print("Commanding robot to stop")
            self.misty.stop()
        else:
            print("Linear Speed: {}  Angular Speed {}".format(misty_vel_x, misty_angular_z))
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






