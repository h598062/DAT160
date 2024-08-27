from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
import time
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'my_robot_pkg'

    package_path = os.path.join(
        get_package_share_directory(package_name))
    xacro_file = os.path.join(package_path,
                              'urdf/',
                              'my_mobile_robot_sensor.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    my_mobile_robot_description = doc.toxml()
    params = {'robot_description': my_mobile_robot_description,
              'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    node_tf = Node(package="tf2_ros",
                   executable="static_transform_publisher",
                   arguments=[
                       "0", "0", "0", "0", "0", "0", "map", "base_link"])

    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d' + os.path.join(get_package_share_directory(package_name),
                                'config', 'config.rviz')]
    )

    world_file_path = os.path.join(
            get_package_share_directory(package_name),
            'worlds', 'my_world.world')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Spawn the robot using gazebo_ros package.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_mobile_robot'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller",
                   "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        # node_joint_state_publisher_gui,
        node_tf,
        node_rviz,
        start_gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])
