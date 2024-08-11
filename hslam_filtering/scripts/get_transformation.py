#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        
        # Initialize subscribers
        self.hslam_subscriber = self.create_subscription(
            Odometry, 'hslam_data', self.hslam_callback, 10
        )
        self.gps_subscriber = self.create_subscription(
            Odometry, 'gps_data', self.gps_callback, 10
        )
        
        # Initialize publisher
        self.transformation_publisher = self.create_publisher(
            Float64MultiArray, 'transformation_parameters', 10
        )
        
        # Initialize data storage
        self.slam_points = []
        self.gps_points = []

    def hslam_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.slam_points.append([x, y, z])
        
        if len(self.slam_points) > 0 and len(self.gps_points) > 0:
            self.process_and_publish_transformation()

    def gps_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.gps_points.append([x, y, z])
        
        if len(self.slam_points) > 0 and len(self.gps_points) > 0:
            self.process_and_publish_transformation()

    def process_and_publish_transformation(self):
        slam_points = np.array(self.slam_points)
        gps_points = np.array(self.gps_points)
        
        # Compute scale, rotation, and translation
        scaling_factors = compute_scaling_factors(slam_points, gps_points)
        scaled_slam_points = scale_slam_points(slam_points, scaling_factors)
        rotation = compute_rotation_matrix(scaled_slam_points, gps_points)
        transformed_slam_points = rotate_slam_points(scaled_slam_points, rotation)
        translation_vector = get_translation_vector(transformed_slam_points, gps_points)

        # Create and publish the transformation message
        msg = Float64MultiArray()
        msg.data = scaling_factors.tolist() + rotation.as_matrix().flatten().tolist() + translation_vector.tolist()
        self.transformation_publisher.publish(msg)
        self.get_logger().info(f'Published transformation data: {msg.data}')

def compute_scaling_factors(slam_points, gps_points):
    s_x = (gps_points[:, 0].max() - gps_points[:, 0].min()) / (slam_points[:, 0].max() - slam_points[:, 0].min()) if slam_points[:, 0].max() != slam_points[:, 0].min() else 1
    s_y = (gps_points[:, 1].max() - gps_points[:, 1].min()) / (slam_points[:, 1].max() - slam_points[:, 1].min()) if slam_points[:, 1].max() != slam_points[:, 1].min() else 1
    s_z = (gps_points[:, 2].max() - gps_points[:, 2].min()) / (slam_points[:, 2].max() - slam_points[:, 2].min()) if slam_points[:, 2].max() != slam_points[:, 2].min() else 1

    return np.array([s_x, s_y, s_z])

def scale_slam_points(slam_points, scaling_factors):
    return slam_points * scaling_factors

def compute_rotation_matrix(scaled_slam_points, gps_points):
    centroid_slam, centroid_gps = np.mean(scaled_slam_points, axis=0), np.mean(gps_points, axis=0)
    scaled_slam_centered = scaled_slam_points - centroid_slam
    gps_centered = gps_points - centroid_gps

    H = np.dot(scaled_slam_centered.T, gps_centered)

    U, _, Vt = np.linalg.svd(H)
    R_matrix = np.dot(Vt.T, U.T)
    
    if np.linalg.det(R_matrix) < 0:
        Vt[-1, :] *= -1
        R_matrix = np.dot(Vt.T, U.T)

    return R.from_matrix(R_matrix)

def rotate_slam_points(scaled_slam_points, rotation):
    rotation_matrix = rotation.as_matrix()
    return np.dot(scaled_slam_points, rotation_matrix.T)

def get_translation_vector(transformed_slam_points, gps_points):
    translation_vector = gps_points[0] - transformed_slam_points[0]
    return translation_vector

def main(args=None):
    rclpy.init(args=args)
    node = TransformationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
