import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mr_pkg')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    ekf_config = os.path.join(pkg_share, 'config', 'dual_ekf_navsat.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start nodes'
        ),
        
        # EKF node for continuous odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        
        # EKF node for map frame (with GPS)
        Node(
            package='robot_localization',
            executable='ekf_node', 
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
        
        # NavSat transform node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('imu/data', '/vehicle/imu'),
                ('gps/fix', '/vehicle/gps/fix'),
                ('odometry/filtered', 'odometry/global')
            ]
        )
    ])