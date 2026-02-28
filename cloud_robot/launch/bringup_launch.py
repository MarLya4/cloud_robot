#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Usar la ruta directa en src
    pkg_dir = os.path.join(os.environ['HOME'], 'ros2_ws_raspi', 'src', 'cloud_robot')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'URDF_DIRECCION_X.xacro')
    
    # Procesar Xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, os.path.join(pkg_dir, 'config', 'cloud_hardware.yaml')],
        output='screen'
    )

    # Spawner de servo_controller
    servo_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['servo_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        servo_controller_spawner
    ])
