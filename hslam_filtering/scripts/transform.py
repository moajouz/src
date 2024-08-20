#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class TransformNode(Node):
    def __init__(self): 
        super().__init__('transform')

        # Initialize current transformation parameters
        self.scaling_factors = None
        self.rotation_matrix = None
        self.translation_vector = None

        # Initialize subscriber for HSLAM data
        self.hslam_subscriber = self.create_subscription(
            Odometry, 'hslam_data', self.hslam_callback, 10
        )
        
        # Initialize subscriber for transformation parameters
        self.parameters_subscriber = self.create_subscription(
            Float64MultiArray, 'transformation_parameters', self.parameters_callback, 10
        )
        
        # Initialize publisher for transformed data
        self.transformation_publisher = self.create_publisher(
            Odometry, 'transformed_data', 10
        )
    
    def hslam_callback(self, msg):
        # Check if transformation parameters are available
        if self.scaling_factors is None or self.rotation_matrix is None or self.translation_vector is None:
            self.get_logger().warn('Transformation parameters not available yet')
            return
        
        # Extract HSLAM data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Perform the transformation
        transformed_point = self.transform_point([x, y, z])

        # Publish the transformed data
        transformed_msg = Odometry()
        transformed_msg.pose.pose.position.x = transformed_point[0]
        transformed_msg.pose.pose.position.y = transformed_point[1]
        transformed_msg.pose.pose.position.z = transformed_point[2]

        self.transformation_publisher.publish(transformed_msg)

        # Log the published message
        # self.get_logger().info(
        #     f"Published transformed data: Position (x, y, z) = "
        #     f"({transformed_msg.pose.pose.position.x}, "
        #     f"{transformed_msg.pose.pose.position.y}, "
        #     f"{transformed_msg.pose.pose.position.z})"
        # )

    def parameters_callback(self, msg):
        if len(msg.data) != 15:
            self.get_logger().error('Received invalid transformation parameters')
            return

        # Extract scaling factors (s_x, s_y, s_z)
        self.scaling_factors = np.array(msg.data[0:3])

        # Extract rotation matrix (3x3)
        self.rotation_matrix = np.array(msg.data[3:12]).reshape((3, 3))

        # Extract translation vector (t_x, t_y, t_z)
        self.translation_vector = np.array(msg.data[12:15])

        self.get_logger().info('received transformation')

    def transform_point(self, point):
        # Scale the point
        scaled_point = np.array(point) * self.scaling_factors

        # Rotate the point
        rotated_point = np.dot(scaled_point, self.rotation_matrix.T)

        # Translate the point
        transformed_point = rotated_point + self.translation_vector

        return transformed_point

def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
