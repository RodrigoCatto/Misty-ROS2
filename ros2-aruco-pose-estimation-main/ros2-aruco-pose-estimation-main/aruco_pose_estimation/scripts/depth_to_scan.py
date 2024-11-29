#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Twist, PoseArray, TransformStamped, TwistWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan, Image
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from nav_msgs.msg import OccupancyGrid, MapMetaData

import os


class DepthToScan(Node):

    def __init__(self):
        super().__init__('depth_to_scan_node')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.get_logger().info('Publishing costmap')

        self.subscription = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

        self.plot_save_dir = "/home/aimsjetson/misty_ws/src/ros2-aruco-pose-estimation-main/ros2-aruco-pose-estimation-main/aruco_pose_estimation/scripts/"
        os.makedirs(self.plot_save_dir, exist_ok=True)  # Ensure the folder exists
        
        self.resolution = 0.05  # Grid resolution in meters/pixel
        self.origin = [-1.0, -1.0, 0.0]  # Costmap origin [x, y, yaw]
        self.downscale_factor = 0.5

        self.saved_once = False

    def listener_callback(self, data):
        """
        Callback function.
        """
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        height, width = depth_image.shape

        ceil_depth = 500    # Minimum depth (e.g., 500 mm)
        floor_depth = 1950   # Maximum depth (e.g., 5000 mm)

        filtered_depth_image = np.where(
            (depth_image >= ceil_depth) & (depth_image <= floor_depth),
            1, # Set 1 for true
            0  # Set 0 for false
        )

        # scaled_down_image = cv2.resize(
        #     filtered_depth_image,
        #     (0, 0),  # Automatically calculate dimensions
        #     fx=self.downscale_factor,  # Scale factor for width
        #     fy=self.downscale_factor,  # Scale factor for height
        #     interpolation=cv2.INTER_AREA  # Use area-based interpolation for downscaling
        # )
        #TODO scale the image to match the actual size it should be
        #TODO change the Nav2 parameters to use this costmap

        occupancy_grid = self.generate_occupancy_grid(filtered_depth_image)
        self.publisher_.publish(occupancy_grid)


        filtered_depth_image = filtered_depth_image.astype(np.uint8)

        # contours, hierarchy = cv2.findContours(filtered_depth_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print("Number of contours: {}".format(len(contours)))

        # size_threshold = 30  # Adjust this value as needed
        # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > size_threshold]
        # filtered_contours = []
        # self.Cxs = []
        # self.Cys = []

        # for contour in contours:
        #     if cv2.contourArea(contour) > size_threshold:
        #         M = cv2.moments(contour)
        #         Cx = int(M['m10']/M['m00'])
        #         Cy = int(M['m01']/M['m00'])
        #         self.Cxs.append(Cx)
        #         self.Cys.append(Cy)
        #         filtered_contours.append(contour)

        
        # cv2.drawContours(depth_image, filtered_contours, -1, (255,255,255), 3)
        # self.plot_images(depth_image, filtered_depth_image)


        # For each blob or countour spread random points towards the blob or towards the contour it self

        #self.odometry.header.stamp = current_time
        #self.publisher_.publish(self.odometry) #Publish scan data
    
    def generate_occupancy_grid(self, filtered_image):
            """
            Convert the filtered depth image to an OccupancyGrid message.
            """
            filtered_image = cv2.flip(filtered_image, 0)
            height, width = filtered_image.shape
            occupancy_data = filtered_image.flatten() * 100  # Scale to [0, 100]

            # Set unknown values (-1) for undefined areas
            # occupancy_data[filtered_image.flatten() == 0] = 0
            # occupancy_data[filtered_image.flatten() == 1] = 1

            # Create OccupancyGrid message
            occupancy_grid = OccupancyGrid()
            occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            occupancy_grid.header.frame_id = 'map'

            # Set metadata
            metadata = MapMetaData()
            metadata.resolution = self.resolution
            metadata.width = width
            metadata.height = height
            metadata.origin.position.x = self.origin[0]
            metadata.origin.position.y = self.origin[1]
            metadata.origin.position.z = 0.0

            (x, y, z, w) = quaternion_from_euler(0, 0, self.origin[2])
            metadata.origin.orientation.x = x
            metadata.origin.orientation.y = y
            metadata.origin.orientation.z = z
            metadata.origin.orientation.w = w

            occupancy_grid.info = metadata

            # Assign occupancy data
            occupancy_grid.data = occupancy_data.tolist()

            return occupancy_grid


    def plot_images(self, depth_image, perpendicular_distances):
        """
        Plot the original depth image and the perpendicular distances.
        """
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))

        # Original depth image
        axes[0].imshow(depth_image, cmap='hsv') # viridis
        axes[0].set_title("Original Depth Image (mm)")
        axes[0].axis('off')

        # Perpendicular distances
        axes[1].imshow(perpendicular_distances, cmap='gray')
        axes[1].plot(self.Cxs, self.Cys, 'r*')
        axes[1].set_title("Perpendicular Distances (mm)")
        axes[1].axis('off')

        # plt.tight_layout()
        # plt.show()

        # Save the figure
        if self.saved_once == False:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.plot_save_dir, f"depth_to_scan_{timestamp}.png")
            plt.savefig(save_path)
            self.get_logger().info(f"Saved plot to {save_path}")
            plt.close(fig)
            self.saved_once = True


def main(args=None):
    rclpy.init(args=args)
    depth_to_scan = DepthToScan()
    rclpy.spin(depth_to_scan)
    depth_to_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

