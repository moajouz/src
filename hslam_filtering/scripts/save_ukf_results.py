#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class UKFDataLogger(Node):
    def __init__(self):
        super().__init__('ukf_data_logger')

        # Subscription to the /odometry/filtered topic (UKF data)
        self.ukf_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.ukf_callback,
            10
        )
        # Subscription to the /gps_data topic (GPS data)
        self.gps_subscription = self.create_subscription(
            Odometry,
            '/gps_data_sync',
            self.gps_save,
            10
        )
        # Subscription to the /transformed_data topic (SLAM data)
        self.slam_subscription = self.create_subscription(
            Odometry,
            '/hslam_data_sync',
            self.slam_save,
            10
        )

        # File paths for saving data
        self.file_path = '/home/mooo/aub/datasets/ficosa_for_HSLAM/new_camera/Auto_EKF/ekf_result_2_may.txt'
        self.merged_path = '/home/mooo/aub/datasets/ficosa_for_HSLAM/new_camera/Auto_EKF/merged_output_2_may.txt'

        try:
            # Open the files in write mode to clear existing data
            self.ukf_data_file = open(self.file_path, 'w')
            self.merged_data_file = open(self.merged_path, 'w')
        except IOError as e:
            self.get_logger().error(f"Failed to open file: {e}")
            raise

    def ukf_callback(self, msg):
        try:
            # Write the UKF data to the file
            self.ukf_data_file.write(
                f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}, "
                f"Position: [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}]\n"
            )
        except IOError as e:
            self.get_logger().error(f"Failed to write UKF data: {e}")

    def gps_save(self, msg):
        try:
            # Write the GPS data to the merged file
            self.merged_data_file.write(
                f"1 {msg.header.stamp.sec}{msg.header.stamp.nanosec} {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z}\n"
            )
        except IOError as e:
            self.get_logger().error(f"Failed to write GPS data: {e}")

    def slam_save(self, msg):
        try:
            # Write the SLAM data to the merged file
            self.merged_data_file.write(
                f"0 {msg.header.stamp.sec}{msg.header.stamp.nanosec} {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z}\n"
            )
        except IOError as e:
            self.get_logger().error(f"Failed to write SLAM data: {e}")

    def destroy_node(self):
        # Close the files before shutting down the node
        if not self.ukf_data_file.closed:
            self.ukf_data_file.close()
        if not self.merged_data_file.closed:
            self.merged_data_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    ukf_data_logger = UKFDataLogger()

    try:
        rclpy.spin(ukf_data_logger)
    except KeyboardInterrupt:
        pass
    finally:
        ukf_data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
