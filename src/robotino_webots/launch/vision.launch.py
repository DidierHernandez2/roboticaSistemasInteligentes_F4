#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # --- Package paths ---
    robotino_webots_share = get_package_share_directory('robotino_webots')
    sim_teleop_launch = os.path.join(
        robotino_webots_share,
        'launch',
        'sim_teleop_joy.launch.py'
    )

    # --- Launch arguments ---
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')

    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/kinect/rgb/image_raw',
        description='RGB image topic for vision nodes'
    )

    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/kinect/depth/image_raw',
        description='Depth pointcloud depthimg topic'
    )

    # --- Webots simulation + teleop ---
    sim_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_teleop_launch)
    )

    # --- Vision node ---
    vision_node = Node(
        package='vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic
        }]
    )

    # --- YOLO server ---
    yolo_server = Node(
        package='vision',
        executable='yolo_server',
        name='yolo_server',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'depth_topic': depth_topic
        }]
    )

    # --- FACE RECOG server ---
    face_recog_server = Node(
        package='vision',
        executable='face_recog_service_node',
        name='face_recog_service_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'depth_topic': depth_topic
        }]
    )

    # --- POSE DETECT server ---
    pose_service_node = Node(
        package='vision',
        executable='pose_service_node',
        name='pose_service_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'depth_topic': depth_topic
        }]
    )

    return LaunchDescription([
        declare_image_topic,
        declare_depth_topic,
        sim_teleop,
        vision_node,
        yolo_server,
        face_recog_server,
        pose_service_node,
    ])
