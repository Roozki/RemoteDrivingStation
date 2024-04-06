import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
#\_SB_.PCI0.XHC_.RHUB.HS01-1.3:1.0-4c4a:4a55
#\_SB_.PCI0.XHC_.RHUB.HS01-1.4:1.0-4c4a:4a55
#\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55

#rover cam in yellow F:  \_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-05a3:9230

def generate_launch_description():
        # Video stream QoS profile
    video_qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=33_333_333  # 30 FPS, expressed in nanoseconds (1/30s)
    )

    rear_feed = Node(
        package='camera_ros',
        executable='camera_node',
        #name='rear_feed_node',
        output='screen',
        #qos=video_qos_profile,
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT1-1:1.0-4c4a:4a55'}], #on g15 direct 
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15 hub 
        #parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'}],
        parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'},
		    {'format': 'MJPEG'},
		    {'height': 480},
		    {'width' : 720}], #onboard hub
	#    parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS04-4:1.0-1bcf:2b8a'},
        #            {'format': 'MJPEG'}], #webcam
        remappings=[
            ('/camera/image_raw', '/vehicle_1/rear_feed/image_raw'),  
            ('/camera/image_raw/compressed', '/vehicle_1/rear_feed/image_raw/compressed'),
            # Add more remappings here if needed
        ],
    )
    #\_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-4c4a:4a55
    main_feed = Node(
        package='camera_ros',
        executable='camera_node',
        #qos=video_qos_profile,
        #name='main_feed_node',
        parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-05a3:9230'},
                    {'format': 'MJPEG'},
                    {'height': 1080},
                    {'width' : 1920}], #onboard hub
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15
	output='screen',
        remappings=[
            ('/camera/image_raw', '/vehicle_1/main_feed/image_raw'),  
            ('/camera/image_raw/compressed', '/vehicle_1/main_feed/image_raw/compressed'),
            # Add more remappings here if needed
        ],
    )
    gnss_serial_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[{'port': '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00'}, {'frame_id': 'gps'}],

    )
    hardware_interface = Node(
        package='rds_hw_interfaces',
        executable='vehicle_interface_node',
        output='screen',
    )
    gps_wgs84_initilizer = Node(
        package='swri_transform_util',
        executable='initialize_origin.py',
        output='screen',
    )
    

    return LaunchDescription([
        main_feed,
        gnss_serial_driver,
        hardware_interface,
        gps_wgs84_initilizer,
        rear_feed
    ])
