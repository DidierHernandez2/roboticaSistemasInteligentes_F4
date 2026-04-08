#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_webots(context, *args, **kwargs):
    world_path = LaunchConfiguration('world_file').perform(context)
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False
    )
    return [webots]


def generate_launch_description():
    pkg_share = get_package_share_directory("robotino_webots")

    declare_world = DeclareLaunchArgument(
        'world_file',

        default_value=os.path.join(pkg_share, 'worlds', 'ABB_inverse_kinematics.wbt'),

        description='Path to Webots world file'
    )

    urdf_path = os.path.join(pkg_share, 'urdf', 'abb_arm.urdf')
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='abb_arm_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path, 'r').read()
            }],
        )
    return LaunchDescription([
        declare_world,
        OpaqueFunction(function=_launch_webots),
        robot_state_publisher_node,
    ])
