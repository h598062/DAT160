from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hello_pub = Node(
        package="my_package",
        executable="my_publisher",  # We defined in the setup.py
        name="my_publisher_from_node"  # new name of the node if you want
    )

    hello_sub = Node(
        package="my_package",
        executable="my_subscriber",
        name="my_subscriber_from_node"
    )

    return LaunchDescription([hello_pub, hello_sub])
