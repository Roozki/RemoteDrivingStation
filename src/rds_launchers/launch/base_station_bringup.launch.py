import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('rds_description'), 'urdf', 'gps_marker_cybertruck.urdf.xacro')

    rviz_config_file = os.path.join(get_package_share_directory('rds_configs'), 'config', 'rviz_config.rviz')
    robot_gps_marker = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_gps_marker_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
    )
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
                      'autorepeat_rate' : 20.0,  
                      'coalesce_interval' : 0.05, 
                      }]
    )
    rds_hud_node = Node(
        package='rds_hud',
        executable='hud_node',
        output='log'
    )
    gps_accuracy_pubber = Node(
        package='rds_gps_py',
        executable='marker_publisher',
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
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'marker_base_link'],
            # arguments are: x, y, z, roll, pitch, yaw, frame_id, child_frame_id
        )
    speaker_node = Node(
        package='rover_sounds',
        executable='speaker_node',
        name='speaker_node',
        output='screen'
    )

    return LaunchDescription([
        gps_accuracy_pubber,
        robot_gps_marker,
        rds_g29_control,
        joy_node,
        rds_hud_node,
        map_node,
        robot_locker,
        speaker_node
    ])