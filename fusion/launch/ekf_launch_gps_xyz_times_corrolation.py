import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        # Static Transform Publisher
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'gps'],
            output='screen'
        ),
        
        # UKF Node
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/fusion/config/ukf_params_gps_xyz_times_corrolation.yaml']
        ),
        launch_ros.actions.Node(
            package='fusion',
            executable='mixed_data_publisher_odom_times_corrolation',
            name='mixed_data_publisher_odom_times_corrolation',
            output='screen'
        )
    ])
