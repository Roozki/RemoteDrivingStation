import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'models/urdf/test_model.urdf'
    
  racetrack_sdf_file_name = 'worlds/racetrack_test.sdf'


  print("urdf_file_name : {}".format(urdf_file_name))



  urdf = os.path.join(
      get_package_share_directory('rds_sims'),
      urdf_file_name)
      
  racetrack_sdf = os.path.join(
    get_package_share_directory('rds_sims'),
    racetrack_sdf_file_name)

  print("sdf_file_name : {}".format(racetrack_sdf))

  pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

  gz_sim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
      launch_arguments={
          'gz_args': racetrack_sdf
          # 'world': racetrack_sdf
      }.items(),
  )
  bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
               '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
    parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable',
                  'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'}],
    output='screen'
)


  return LaunchDescription([
        # LogInfo(msg="Launching Prius on a racetrack using Ignition Gazebo..."),
        # ExecuteProcess(
        #     cmd=['ign', 'gazebo', racetrack_sdf_file_name, '--verbose'],
        #     output='screen'
        # ),
        gz_sim

    #     DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='false',
    #         description='Use simulation (Gazebo) clock if true'),

    #    ExecuteProcess(
    #         cmd=['ign', 'gazebo', '--verbose', ],
    #         output='screen'),

    #     Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         arguments=[urdf]),

    #     Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         name='joint_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time}]
    #         ),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     name='urdf_spawner',
        #     output='screen',
        #     arguments=["-topic", "/robot_description", "-entity", "cam_bot"])
  ])