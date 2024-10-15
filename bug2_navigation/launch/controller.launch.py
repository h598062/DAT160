import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="bug2_navigation",
                executable="wall_follower",
                name="wall_follower",
            ),
            launch_ros.actions.Node(
                package="bug2_navigation", executable="go_to_point", name="go_to_point"
            ),
            launch_ros.actions.Node(
                package="bug2_navigation",
                executable="bug2_navigation",
                name="bug2_navigation",
            ),
            # launch_ros.actions.Node(
            #     package='bug2_navigation',
            #     executable='robot_controller',
            #     name='robot_controller'),
        ]
    )
