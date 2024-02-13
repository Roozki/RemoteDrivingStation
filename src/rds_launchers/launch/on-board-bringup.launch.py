import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    camera_node_1 = Node(
        package='camera_ros',
        executable='camera_node',
        output='screen',
        parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS04-4:1.0-1bcf:2b8a'}]
    )
    camera_node_2 = Node(
        package='camera_ros',
        executable='camera_node',
        output='screen'
    )

    return LaunchDescription([
        camera_node_1
       #camera_node_2
    ])
