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
        remappings=[
            ('/camera/image_raw', '/vehicle_1/rear_feed/image_raw'),  # Remap from /image_raw to /camera1/image_raw
            ('/camera/image_raw/compressed', '/vehicle_1/rear_feed/image_raw/compressed'),
            # Add more remappings here if needed
        ],
    )
    camera_node_2 = Node(
        package='camera_ros',
        executable='camera_node',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/vehicle_1/main_feed/image_raw'),  # Remap from /image_raw to /camera1/image_raw
            # Add more remappings here if needed
        ],
    )

    return LaunchDescription([
        camera_node_2
       #camera_node_2
    ])
