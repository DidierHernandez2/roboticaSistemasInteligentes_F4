from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Top camera (kinect) topics published by robotino_webots_controller
    image_topic = '/kinect_sim/rgb/image_raw'
    depth_topic = '/kinect_sim/depth/image_raw'
    depth_info_topic = '/kinect_sim/depth/camera_info'

    return LaunchDescription([

        # 1. YOLO server — subscribes to kinect, exposes /yolo_detect service
        Node(
            package='vision',
            executable='yolo_server',
            name='yolo_server',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'depth_topic': depth_topic,
                'depth_info_topic': depth_info_topic,
                'model_path': 'yolo11n-seg.pt',  # downloads automatically on first run
            }]
        ),

        # 2. Continuous person detection node — calls service, shows annotated feed
        Node(
            package='vision',
            executable='yolo_person_node',
            name='yolo_person_node',
            output='screen',
            parameters=[{
                'display': True,
                'interval': 0.3,
                'spin_speed': 0.4,  # rad/s, positive = counterclockwise
            }]
        ),
    ])
