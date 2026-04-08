from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

WEBOTS_HOME = '/snap/webots/current/usr/share/webots'
WEBOTS_CONTROLLER_LIB = '/snap/webots/current/usr/share/webots/lib/controller/python'

def generate_launch_description():
    # Get package share directories
    robotino_webots_share = get_package_share_directory('robotino_webots')
    
    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    
    # Load the Webots URDF - FIXED PATH
    urdf_path = os.path.join(
        robotino_webots_share,
        'urdf',
        'Robotino3.urdf'  # Using the actual file name
    )
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        SetEnvironmentVariable(name='WEBOTS_HOME', value=WEBOTS_HOME),
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=WEBOTS_CONTROLLER_LIB + ':' + os.environ.get('PYTHONPATH', '')
        ),

        # Launch argument for world file
        DeclareLaunchArgument(
            'world_file',
            default_value=get_package_share_directory('robotino_webots') + '/worlds/robotino_apartment.wbt',
            description='Path to Webots world file'
        ),
        
        # Webots world
        ExecuteProcess(
            cmd=['webots', world_file],
            output='screen'
        ),
        
        # Robot controller
        Node(
            package='robotino_webots',
            executable='robotino_webots_controller.py',
            name='robotino_controller',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        
        # ROBOT STATE PUBLISHER - WITH CORRECT URDF PATH
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
                'publish_frequency': 30.0
            }]
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='kinect_optical_tf',
            arguments=[
                '0', '0', '0',              # x y z
                '-0.5', '0.5', '-0.5', '0.5',  # qx qy qz qw
                'kinect_link',              # parent frame
                'kinect_optical'            # child frame
            ],
            output='screen'
        ),
        
    ])