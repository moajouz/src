#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Synchronization(Node):
    def __init__(self):
        super().__init__('get_transformation')
        
        # Initialize subscribers
        self.hslam_subscriber = self.create_subscription(
            Odometry, 'transformed_data', self.hslam_callback, 10
        )
        self.gps_subscriber = self.create_subscription(
            Odometry, 'gps_data', self.gps_callback, 10
        )   
        
        # Initialize publishers
        self.gps_sync = self.create_publisher(
            Odometry, 'gps_data_sync', 10
        )
        self.hslam_sync = self.create_publisher(
            Odometry, 'hslam_data_sync', 10
        )   

        # Initialize counters and data holders
        self.slam_counter = 0
        self.gps_counter = 0
        self.gps_freq = 10000
        self.gps_data = None
        self.slam_data = None

    def hslam_callback(self, msg):
        self.slam_data = msg
        # self.slam_counter += 1
        # self.check_and_publish()
        self.hslam_sync.publish(self.slam_data)
        self.get_logger().info(f'sent slam:{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}')


    def gps_callback(self, msg):
        self.gps_data = msg
        # self.gps_counter += 1
        # self.check_and_publish()
        self.gps_sync.publish(self.gps_data)
        self.get_logger().info(f'sent gps:{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}')

    # def check_and_publish(self):
    #     if self.slam_data and self.gps_data:
    #         # Assuming the logic here is to publish both synchronized messages
    #         self.hslam_sync.publish(self.slam_data)
    #         self.gps_sync.publish(self.gps_data)
            
    #         # Reset data
    #         self.slam_data = None
    #         self.gps_data = None

    #     elif self.gps_counter > self.gps_freq and not self.slam_data:
    #         # Publish GPS data if no SLAM data is available for a while
    #         self.gps_sync.publish(self.gps_data)
    #         self.gps_data = None
    #         self.gps_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = Synchronization()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
