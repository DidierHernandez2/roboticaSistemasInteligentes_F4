from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to teleop YAML inside robotino_webots/config/
    config_file = os.path.join(
        get_package_share_directory('robotino_webots'),
        'config',
        'teleop_joy_robotino.yaml'
    )

    return LaunchDescription([

        # ------------------------------------
        # 1. JOYSTICK DRIVER
        # ------------------------------------
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'device': '/dev/input/js0'},
                {'deadzone': 0.05},
                {'autorepeat_rate': 20.0},
            ],
        ),

        # ------------------------------------
        # 2. TELEOP TWIST JOY
        # ------------------------------------
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('cmd_vel', '/cmd_vel')
            ]
        ),
    ])
