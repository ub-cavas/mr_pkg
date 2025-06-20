from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch your talker node
        Node(
            package='mr_pkg',
            executable='world_transform',
            name='world_transform_node',
            output='screen',
        ),
        # Launch your listener node
        Node(
            package='mr_pkg',
            executable='virtual_perception',
            name='virtual_perception_node',
            output='screen',
        ),
        # Add more Node(...) entries here as needed
    ])