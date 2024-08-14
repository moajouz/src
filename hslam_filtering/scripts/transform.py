#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform')

        # Initialize data storage
        self.slam_points = []
        self.timestamps = []

        # Initialize subscribers
        self.hslam_subscriber = self.create_subscription(
            Odometry, 'hslam_data', self.hslam_callback, 10
        )
        self.gps_subscriber = self.create_subscription(
            Float64MultiArray, 'transformation_parameters', self.parameters_callback, 10
        )
        
        # Initialize publisher
        self.transformation_publisher = self.create_publisher(
            Odometry, 'transformed_data', 10
        )
    
    def hslam_callback(self, msg):
        timestamp = msg.header.stamp
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.slam_points.append([x, y, z])
        self.timestamps.append(timestamp)

    def parameters_callback(self, msg):
        # Extract scaling factors (s_x, s_y, s_z)
        self.scaling_factors = np.array(msg.data[0:3])

        # Extract rotation matrix (3x3)
        self.rotation_matrix = np.array(msg.data[3:12]).reshape((3, 3))

        # Extract translation vector (t_x, t_y, t_z)
        self.translation_vector = np.array(msg.data[12:15])

    def scale_slam_points(self, slam_points, scaling_factors):
        return np.array(slam_points) * scaling_factors
    
    #rotate scaled points
    def rotate_slam_points(self, scaled_slam_points, rotation):
        rotation_matrix = rotation.as_matrix()
        return np.dot(scaled_slam_points, rotation_matrix.T)
    
    #compute transformation
    def transform(self):
        if len(self.slam_points) > 0 and len(self.timestamps) > 0:
            # Create Odometry message
            msg = Odometry()
            
            # Compute transformation for the first set of points
            scaled_value = self.scale_slam_points(self.slam_points[0], self.scaling_factors)
            rotated_value = self.rotate_slam_points(scaled_value, self.rotation_matrix)
            transformed_slam = rotated_value + self.translation_vector
            
            # Set message
            msg.header.stamp = self.timestamps[0]
            msg.pose.pose.position.x = transformed_slam[0]
            msg.pose.pose.position.y = transformed_slam[1]
            msg.pose.pose.position.z = transformed_slam[2]
            
            # Log the published message
            self.get_logger().info(
                f"Published transformed data: "
                f"Position (x, y, z) = ({msg.pose.pose.position.x}, "
                f"{msg.pose.pose.position.y}, {msg.pose.pose.position.z}), "
                f"Timestamp = {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
            )
            
            # Publish the transformed data
            self.transformation_publisher.publish(msg)
            
            # Remove processed data from arrays
            self.timestamps.pop(0)
            self.slam_points.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()