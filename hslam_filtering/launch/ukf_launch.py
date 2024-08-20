from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'gps'],
            output='screen'
        ),

        # Launch the save_ukf_results.py Python script
        Node(
            package='hslam_filtering',
            executable='save_ukf_results.py',
            name='save_ukf_results',
            output='screen'
        ),

        # UKF Node
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/hslam_filtering/config/ukf_params.yaml']
        ),

        # Launch the data_control.py Python script
        Node(
            package='hslam_filtering',
            executable='data_control.py',
            name='data_control',
            output='screen'
        ),

        # Launch the transform.py Python script
        Node(
            package='hslam_filtering',
            executable='transform.py',
            name='transform',
            output='screen'
        ),

        # Launch the get_transformation.py Python script
        Node(
            package='hslam_filtering',
            executable='get_transformation.py',
            name='get_transformation',
            output='screen'
        ),

        # Launch the data_publisher C++ node
        Node(
            package='hslam_filtering',
            executable='data_publisher',
            name='data_publisher',
            output='screen'
        ),
    ])
