import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rds_g29_control = Node(
        package='rds_control',
        executable='g29_hw_control',
        output='screen',
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate' : 20.0, 'coalesce_interval_ms' : 500}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
        #parameters=[{'config' : }]
    )

    return LaunchDescription([
        rds_g29_control,
        joy_node,
        rviz_node
    ])