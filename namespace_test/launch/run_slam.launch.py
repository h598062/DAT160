import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="namespace_test",
                executable="tb_scan",
                namespace="",
                name="tb_scan",
            ),
            launch_ros.actions.Node(
                package="namespace_test",
                executable="slam",
                namespace="",
                name="slam",
            ),
        ]
    )
