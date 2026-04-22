from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='l510_controller',
            executable='l510_node',
            name='l510_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'slave': 1,
                'baudrate': 9600,
                'startup_hz': 10.0,
                'auto_run': False,
                'reverse': False,
            }]
        )
    ])
