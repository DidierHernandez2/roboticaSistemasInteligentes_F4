#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robotino_webots_share = get_package_share_directory('robotino_webots')
    robotino_audio_share = get_package_share_directory('robotino_audio')
    rplidar_share = get_package_share_directory('rplidar_ros')

    # --- Paths to launches (vision/audio kept, nav swapped to real) ---
    real_localization_launch = os.path.join(robotino_webots_share, 'launch', 'real_robo_localization.launch.py')
    real_nav2_launch = os.path.join(robotino_webots_share, 'launch', 'real_robo_nav2.launch.py')

    vision_launch = os.path.join(robotino_webots_share, 'launch', 'vision.launch.py')
    speech_launch = os.path.join(robotino_audio_share, 'launch', 'speech_recog.launch.py')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar.launch.py')

    # --- Launch arguments (mirror sim structure + real additions) ---
    hostname = LaunchConfiguration('hostname')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    declare_hostname = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.0.1:12080',
        description='Robotino hostname:port (e.g. 192.168.0.1:12080)'
    )

    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(os.path.expanduser('~'), 'maps', 'map_tec_2.yaml'),
        description='Path to map YAML file'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real robot)'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart Nav2 lifecycle nodes'
    )

    # Keep same vision args as sim (so BT/vision code doesn’t care)
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/kinect/image_raw',
        description='RGB image topic for vision nodes'
    )

    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/kinect/depth/image_raw',
        description='Depth topic for vision nodes'
    )

    # Lidar args (exactly your CLI)
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='RPLidar serial baudrate'
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='RPLidar frame_id'
    )

    # --- Includes ---
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id
        }.items()
    )

    include_real_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_localization_launch),
        launch_arguments={
            'hostname': hostname,
            'map_file': map_file
        }.items()
    )

    include_real_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_nav2_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    include_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch),
        launch_arguments={
            'image_topic': image_topic,
            'depth_topic': depth_topic
        }.items()
    )

    include_speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(speech_launch),
        launch_arguments={}.items()
    )

    # Keep this TF glue exactly as in sim launch (vision convenience)
    kinect_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_depth_tf_glue',
        arguments=['0', '0', '0', '0', '0', '0', 'kinect_link', 'kinect_depth'],
        output='screen'
    )

  
    return LaunchDescription([
        declare_hostname,
        declare_map,
        declare_sim_time,
        declare_autostart,
        declare_image_topic,
        declare_depth_topic,
        declare_serial_port,
        declare_serial_baudrate,
        declare_frame_id,
        #################################
        include_rplidar,
        TimerAction(period=1.0, actions=[include_real_localization]),
        TimerAction(period=2.0, actions=[include_real_nav2]),
        ####################################
        include_vision,
        kinect_depth_tf,
        include_speech,
    ])
