import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Path to the dual_ekf.launch.py file in mr_pkg
    dual_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mr_pkg'),
                'launch',
                'dual_ekf.launch.py'
            ])
        ])
    )

    sensor_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mr_pkg'),
                'launch',
                'static_transforms.launch.py'
            ])
        ])
    )

    

    return LaunchDescription([
        dual_ekf_launch,
        sensor_transform_launch
    ])