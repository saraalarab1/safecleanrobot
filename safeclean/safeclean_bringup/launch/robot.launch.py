import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='safeclean_control',
        executable='motor_controller',
        parameters=[],
        remappings=[],
        arguments=[],
    )

    ld.add_action(node)

    return ld
