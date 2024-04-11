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
    front_cam_params_v4l2 = os.path.join(
    get_package_share_directory('rds_launchers'),
    'config', 'front_cam_params_v4l2.yaml'
    )
    rear_cam_params_v4l2 = os.path.join(
    get_package_share_directory('rds_launchers'),
    'config', 'rear_cam_params_v4l2.yaml'
    )
    front_cam_params = os.path.join(
    get_package_share_directory('rds_launchers'),
    'config', 'front_cam_params_camros.yaml'
    )
    rear_cam_params = os.path.join(
    get_package_share_directory('rds_launchers'),
    'config', 'rear_cam_params.yaml'
    )
        # Video stream QoS profile
    video_qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        #deadline=33_333_333  # 30 FPS, expressed in nanoseconds (1/30s)
    )

    rear_feed = Node(
        package='camera_ros',
        executable='camera_node',
        name='rear_camera_node',
        output='screen',
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT1-1:1.0-4c4a:4a55'}], #on g15 direct 
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15 hub 
        #parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'}],
        parameters=[rear_cam_params],
        # parameters=[
        #     {'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'},
		#     {'format': 'MJPEG'},
		#     {'height': 480},
		#     {'width' : 720}], #onboard hub
	#    parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS04-4:1.0-1bcf:2b8a'},
        #            {'format': 'MJPEG'}], #webcam
        remappings=[
            ('/rear_camera_node/image_raw', '/vehicle_1/rear_feed/image_raw'),  
            ('/rear_camera_node/image_raw/compressed', '/vehicle_1/rear_feed/image_raw/compressed'),
            ('/rear_camera_node/camera_info', '/vehicle_1/rear_feed/camera_info'),
            # Add more remappings here if needed
        ],
    )
    # \_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-4c4a:4a55
    main_feed = Node(
        package='camera_ros',
        executable='camera_node',
        #qos_overrides=video_qos_profile,
        name='front_camera_node',
        parameters=[front_cam_params],
        # parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-05a3:9230'},
        #             {'format': 'MJPEG'},
        #             {'height': 1080},
        #             {'width' : 1920}], #onboard hub
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15
	    output='screen',
        remappings=[
            ('/front_camera_node/image_raw', '/vehicle_1/main_feed/image_raw'),  
            ('/front_camera_node/image_raw/compressed', '/vehicle_1/main_feed/image_raw/compressed'),
            ('/front_camera_node/camera_info', '/vehicle_1/main_feed/camera_info'),

            # Add more remappings here if needed
        ],
    )

    rear_feed_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='rear_camera_node',
        output='screen',
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT1-1:1.0-4c4a:4a55'}], #on g15 direct 
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15 hub 
        #parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'}],
        parameters=[rear_cam_params],
        # parameters=[
        #     {'camera': '\_SB_.PCI0.XHC_.RHUB.HS01-1:1.0-4c4a:4a55'},
		#     {'format': 'MJPEG'},
		#     {'height': 480},
		#     {'width' : 720}], #onboard hub
	#    parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS04-4:1.0-1bcf:2b8a'},
        #            {'format': 'MJPEG'}], #webcam
        remappings=[
            ('/image_raw', '/vehicle_1/rear_feed/image_raw'),  
            ('/image_raw/compressed', '/vehicle_1/rear_feed/image_raw/compressed'),
            ('/camera_info', '/vehicle_1/rear_feed/camera_info'),
            # Add more remappings here if needed
        ],
    )
    main_feed_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        #qos_overrides=video_qos_profile,
        name='front_camera_node',
        parameters=[front_cam_params],
        # parameters=[{'camera': '\_SB_.PCI0.XHC_.RHUB.HS02-2:1.0-05a3:9230'},
        #             {'format': 'MJPEG'},
        #             {'height': 1080},
        #             {'width' : 1920}], #onboard hub
        #parameters=[{'camera': '\_SB_.PCI0.GP17.XHC0.RHUB.PRT2-2.3:1.0-4c4a:4a55'}], #on g15
	    output='screen',
        remappings=[
            ('/image_raw', '/vehicle_1/main_feed/image_raw'),  
            ('/image_raw/compressed', '/vehicle_1/main_feed/image_raw/compressed'),
            ('/camera_info', '/vehicle_1/main_feed/camera_info'),
            ('/image_raw/theora', '/vehicle_1/main_feed/image_raw/theora'),
            ('/image_raw/compressedDepth', '/vehicle_1/main_feed/image_raw/compressedDepth')

            # Add more remappings here if needed
        ],
    )
    # main_feed_compression_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     parameters=[{'ffmpeg_image_transport.encoding': 'hevc_nvenc',
    #             'ffmpeg_image_transport.profile': 'main',
    #             'ffmpeg_image_transport.preset': 'll',
    #             'ffmpeg_image_transport.gop': 15}]
    #     remappings=[('/in', '/vehicle_1/main_feed/image_raw'),
    #                 ('/out', '/vehicle_1/main_feed/image_raw/h264')]
    # )
    main_ffmpeg_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='ffmpeg_republisher',
        remappings=[
            ('in/image_raw', '/vehicle_1/main_feed/image_raw'),
            ('out/ffmpeg', '/vehicle_1/main_feed/image_raw/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx265',  # Use H.265 encoding with NVENC (if available)
             'ffmpeg_image_transport.profile': 'baseline',
             'ffmpeg_image_transport.preset': 'ultrafast',            # Low latency preset
             'ffmpeg_image_transport.tune': 'zerolatency',         # Main profile for compatibility
             'ffmpeg_image_transport.gop_size': 5,            # GOP size (group of pictures)
             'ffmpeg_image_transport.bit_rate': 8242880,   # Target bitrate
             'ffmpeg_image_transport.frame_rate' : 30,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/vehicle_1/main_feed/image_raw', 'ffmpeg', 'out:=/vehicle_1/main_feed/image_raw/h265']
    )
    rear_ffmpeg_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='ffmpeg_republisher',
        remappings=[
            ('in/image_raw', '/vehicle_1/rear_feed/image_raw'),
            ('out/ffmpeg', '/vehicle_1/rear_feed/image_raw/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx265',  # Use H.265 encoding with NVENC (if available)
             'ffmpeg_image_transport.profile': 'baseline',
             'ffmpeg_image_transport.preset': 'ultrafast',            # Low latency preset
             'ffmpeg_image_transport.tune': 'zerolatency',         # Main profile for compatibility
             'ffmpeg_image_transport.gop_size': 5,            # GOP size (group of pictures)
             'ffmpeg_image_transport.bit_rate': 4242880,   # Target bitrate
             'ffmpeg_image_transport.frame_rate': 30,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/vehicle_1/rear_feed/image_raw', 'ffmpeg', 'out:=/vehicle_1/rear_feed/image_raw/h265']
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
        # main_feed_v4l2,
        main_feed,
        gnss_serial_driver,
        hardware_interface,
        gps_wgs84_initilizer,
        # rear_feed_v4l2,
        rear_feed,
        rear_ffmpeg_republisher_node,
        main_ffmpeg_republisher_node
    ])
