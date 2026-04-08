#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robotino_webots_share = get_package_share_directory('robotino_webots')
    robotino_audio_share = get_package_share_directory('robotino_audio')

    # Paths to existing launches
    nav_launch = os.path.join(robotino_webots_share, 'launch', 'nav_robotino.launch.py')
    vision_launch = os.path.join(robotino_webots_share, 'launch', 'vision.launch.py')
    speech_launch = os.path.join(robotino_audio_share, 'launch', 'speech_recog.launch.py')

    # launch arguments 
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')

    
    
    declare_world = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(robotino_webots_share, 'worlds', 'robotino_apartment.wbt'),
        description='Path to Webots world file'
    )

    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.expanduser('~/robotec_ws/map_2.yaml'),
        description='Path to map YAML file'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        #default_value='/kinect_sim/rgb/image_raw',
        default_value='/kinect/image_raw',
        description='RGB image topic for vision nodes'
    )

    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        #default_value='/kinect_sim/depth/image_raw',
        default_value='/kinect/depth/image_raw',
        description='Depth pointcloud img  topic'
    )
    

    # Include the nav and vision launch files, forwarding args
    include_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'world_file': world_file,
            'map_file': map_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    include_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch),
        launch_arguments={
            'image_topic': image_topic,
            'depth_topic': depth_topic
        }.items()
    )

    # include_speech = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(speech_launch),
    #     launch_arguments={}.items()
    # )  # disabled: requires audio_capture package
    kinect_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_depth_tf_glue',
        arguments=['0','0','0', '0','0','0', 'kinect_link', 'kinect_depth'],
        output='screen'
    )
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0','0','0', '0','0','0', 'base_footprint', 'base_link'],
        output='screen'
    )
    return LaunchDescription([
        declare_world,
        declare_map,
        declare_sim_time,
        declare_image_topic,
        declare_depth_topic,
        include_nav,
        include_vision,
        kinect_depth_tf,
        base_tf,
        # include_speech,  # disabled: requires audio_capture package
    ])
