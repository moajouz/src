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

        # NavSat Transform Node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/hslam_filtering/config/navsat_transform_params.yaml']
        ),
        
        # UKF Node
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/hslam_filtering/config/ukf_params.yaml']
        ),
        
        # Launch the save_ukf_results.py Python script
        Node(
            package='hslam_filtering',
            executable='save_ukf_results.py',
            name='save_ukf_results',
            output='screen'
        ),
        
        # Launch the data_control.py Python script
        Node(
            package='hslam_filtering',
            executable='data_control.py',
            name='data_control',
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
