# static_transforms.launch.py
#
# Example:
#   ros2 launch <your_pkg> static_transforms.launch.py
#
# All angles are in radians.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # --- map -> odom ---
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # --- odom -> base_link ---
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # --- base_link -> imu_link ---
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link_broadcaster',
        arguments=['0.2', '0.0', '0.3', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # --- base_link -> gnss_link ---
    gnss_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gnss_link_broadcaster',
        arguments=['0.0', '0.0', '1.5', '0', '0', '0', 'base_link', 'gnss_link'],
        output='screen'
    )

    # --- base_link -> velodyne_top ---
    velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne_top_broadcaster',
        arguments=['0.0', '0.0', '1.8', '0', '0', '0', 'base_link', 'velodyne_top'],
        output='screen'
    )

    return LaunchDescription([
        map_tf,
        odom_tf,
        imu_tf,
        gnss_tf,
        velodyne_tf
    ])
