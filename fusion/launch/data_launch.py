import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # GPS Publisher Node
        launch_ros.actions.Node(
            package='fusion',
            executable='gps_publisher',
            name='gps_publisher',
            output='screen'
        ),
        # HSLAM Publisher Node
        launch_ros.actions.Node(
            package='fusion',
            executable='hslam_publisher',
            name='hslam_publisher',
            output='screen'
        )
    ])
