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
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/fusion/config/ekf_params.yaml']
        ),
        launch_ros.actions.Node(
            package='hslam_filtering',
            executable='data_publisher',
            name='data_publisher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='hslam_filtering',
            executable='get_timestamp',
            name='get_timestamp',
            output='screen'
        )
    ])
