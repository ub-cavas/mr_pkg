from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mr_pkg',
            executable='image_overlay',
            name='image_overlay_node',
            output='screen',
            parameters=[],
        ),

        Node(
            package='mr_pkg',
            executable='webcam_publisher',
            name='webcam_publisher_node',
            output='screen',
            parameters=[],
        ),
    ])