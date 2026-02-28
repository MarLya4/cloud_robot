#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    cloud_robot_dir = get_package_share_directory('cloud_robot')

    # Procesar XACRO
    xacro_file = os.path.join(cloud_robot_dir, 'urdf', 'URDF_DIRECCION_X.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo ros2_control con tu YAML de hardware
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(cloud_robot_dir, 'config', 'cloud_hardware.yaml')
        ],
        output='screen'
    )

    # Spawners de controladores
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--timeout', '10'
        ],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--timeout', '10'
        ],
        output='screen'
    )

    servo_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'servo_controller',
            '--controller-manager', '/controller_manager',
            '--timeout', '10'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        #diff_drive_controller_spawner,
        servo_controller_spawner
    ])
