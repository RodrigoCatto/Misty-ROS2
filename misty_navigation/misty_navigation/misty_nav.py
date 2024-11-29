import rclpy
from rclpy.node import Node

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Twist, PoseArray, TransformStamped, TwistWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class MistyNav(Node):

    def __init__(self):
        super().__init__('misty_nav_node')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)   #misty_3/odom
        self.get_logger().info('Publishing odometry info')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(ArucoMarkers, '/aruco/markers', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

        self.markers_data = ArucoMarkers()
        self.misty_vel = Twist()

        self.broadcaster = TransformBroadcaster(self, 10) #odom frame broadcaster
        joint_state = JointState()
        self.odometry = Odometry()  #odom information
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'
        self.tw_msg = TwistWithCovariance()
        self.corrected_orientation = Quaternion()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        markers_data = data
        markers_ids = markers_data.marker_ids
        marker_poses = markers_data.poses
        self.marker_poses = zip(markers_ids, marker_poses)

        for marker in self.marker_poses:
            marker_id = marker[0]
            if marker_id == 6:
                current_time = self.get_clock().now().to_msg()

                self.odometry.pose.pose.position.x = marker[1].position.x
                self.odometry.pose.pose.position.y = -marker[1].position.y
                self.odometry.pose.pose.position.z = 0.0


                # Extract orientation as quaternion and convert to Euler
                x = marker[1].orientation.x
                y = marker[1].orientation.y
                z = marker[1].orientation.z
                w = marker[1].orientation.w

                (roll, pitch, yaw) = euler_from_quaternion((x, y, z, w))

                # Adjust yaw to correct direction
                roll = 0
                pitch = 0
                yaw = - yaw  # Adjust yaw as needed (180-degree rotation)

                # Convert modified Euler angles back to quaternion
                (x_new, y_new, z_new, w_new) = quaternion_from_euler(roll, pitch, yaw)

                # Set the updated orientation in odometry
                self.odometry.pose.pose.orientation.x = x_new
                self.odometry.pose.pose.orientation.y = y_new
                self.odometry.pose.pose.orientation.z = z_new
                self.odometry.pose.pose.orientation.w = w_new

                #(x,y,z,w) = quaternion_from_euler(roll, pitch, yaw)

                # self.odometry.pose.pose.orientation.x = x
                # self.odometry.pose.pose.orientation.y = y
                # self.odometry.pose.pose.orientation.z = z
                # self.odometry.pose.pose.orientation.w = w

                #self.odometry.pose.pose.orientation = marker[1].orientation

                # self.odometry.twist.twist.linear.x =  0.0
                # self.odometry.twist.twist.linear.y =  0.0
                # self.odometry.twist.twist.linear.z =  0.0
                # self.odometry.twist.twist.angular.x = 0.0
                # self.odometry.twist.twist.angular.y = 0.0
                # self.odometry.twist.twist.angular.z = 0.0

                self.odometry.header.stamp = current_time
                self.publisher_.publish(self.odometry)

                self.odom_trans.header.stamp = current_time
                self.odom_trans.transform.translation.x = self.odometry.pose.pose.position.x
                self.odom_trans.transform.translation.y = self.odometry.pose.pose.position.y
                self.odom_trans.transform.translation.z = self.odometry.pose.pose.position.z
                self.odom_trans.transform.rotation = self.odometry.pose.pose.orientation     
                self.broadcaster.sendTransform(self.odom_trans)
                
            #print("Marker ID: {} Pos: {} {}".format(marker[0], marker[1].position, marker[1].position.y))

    def get_position_setpoint(x,y,rot,id):
        pass
    

    def control_misty(self, x_des,y_des,rot_des,des_id, current_pos):
        for marker in self.marker_poses:
            marker_id = marker[0]
            Kp = 1
            x_pos = marker[1].position.x
            y_pos = marker[1].position.y
            if marker_id == des_id:
                x_err = x_des - x_pos
                y_err = y_des - y_pos
            
                x_out = Kp*x_err
                y_out = Kp*y_err

            return [x_out, y_out]
            # print("Marker ID: {} Pos: {} {}".format(marker[0], marker[1].position, marker[1].position.y))


def convert_speed_and_publish(self, vel_x, vel_y):
    self.misty_vel.linear.x = vel_x
    self.misty_vel.linear.y = vel_y
    self.publisher_.publish(self.misty_vel)
    self.get_logger().info('Publishing images')


def main(args=None):
    rclpy.init(args=args)

    misty_nav = MistyNav()

    rclpy.spin(misty_nav)

    misty_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

