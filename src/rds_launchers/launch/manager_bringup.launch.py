import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('rds_configs'), 'config', 'manager_config.rviz')
  

    rds_gui_node = Node(
        package='rds_gui',
        executable='rds_qt',
        output='screen'
    )
    map_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[{'d' : rviz_config_file}]
    )
    robot_locker = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_broadcaster_base_link_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            # arguments are: x, y, z, roll, pitch, yaw, frame_id, child_frame_id
        )

    return LaunchDescription([
        rds_gui_node,
        map_node,
        robot_locker
    ])