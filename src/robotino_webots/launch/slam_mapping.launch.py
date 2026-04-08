from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
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
        # Launch argument for world file
        DeclareLaunchArgument(
            'world_file',
            #default_value='/home/oscar/simple_slam_world.wbt',
            default_value=get_package_share_directory('robotino_webots') + '/worlds/robotino_apartment.wbt',
            # default_value=get_package_share_directory('robotino_webots') + '/worlds/robotino_apartment.wbt',
            
            
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
            parameters=[{'use_sim_time': True}],
        ),
        
        # ROBOT STATE PUBLISHER - WITH CORRECT URDF PATH
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'publish_frequency': 30.0
            }]
        ),
        
        
        
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('robotino_webots'),
                'config',
                'slam_toolbox.yaml'
            ),
            'use_sim_time': 'true'
        }.items()
        ),
       
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}]
        )
    ])