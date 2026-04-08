from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('robotino_webots'),
        'config', 'teleop_joy_robotino.yaml'
    )

    print(f"Using teleop config: {config_file}")

    return LaunchDescription([
        # JOYSTICK DRIVER
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'device': '/dev/input/js0'},
                {'deadzone': 0.05},
                {'autorepeat_rate': 20.0},
            ],
        ),

        # TELEOP TWIST JOY
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
