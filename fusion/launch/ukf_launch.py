import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        # UKF Node
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/mooo/ros2_ws/src/fusion/config/ukf_params.yaml']
        ),
        launch_ros.actions.Node(
            package='fusion',
            executable='mixed_data_publisher',
            name='mixed_data_publisher',
            output='screen'
        )
    ])
