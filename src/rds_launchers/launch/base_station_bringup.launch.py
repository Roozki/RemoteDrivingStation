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
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters=[{
                      'autorepeat_rate' : 32.0,
                      'coalesce_interval' : 0.03125,
                      }]
    )
    rds_hud_node = Node(
        package='rds_hud',
        executable='hud_node',
        output='log'
    )
    rviz_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='log'
        #parameters=[{'config' : }]
    )

    return LaunchDescription([
        rds_g29_control,
        joy_node,
        rds_hud_node
       # rviz_node
    ])