from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Launch arguments
    hostname = LaunchConfiguration('hostname', default='192.168.0.1:12080')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hostname', default_value='192.168.0.1:12080'),
        
        # Launch Robotino driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('rto_node'),
                '/launch/robotino_driver.launch.py'
            ]),
            launch_arguments={
                'hostname': hostname,
                'launch_odom_tf': 'true',
                'namespace': ''
            }.items()
        ),
        
        # Add base_footprint frame (to match simulation if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_footprint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        
        # Add laser frame (THIS WAS MISSING!)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('slam_toolbox'),
                '/launch/online_async_launch.py'
            ]),
            launch_arguments={
            'use_sim_time': 'false',
            'odom_frame': 'odom',
            'base_frame': 'base_link', 
            'scan_topic': 'scan',
            
            # MAXIMUM UPDATE FREQUENCY
            'slam_toolbox.map_update_interval': '0.1',  # Update every 0.1 seconds
            'slam_toolbox.transform_timeout': '0.1',   # Ultra-fast TF lookups
            'slam_toolbox.processing_queue_size': '10',   # No queuing, process immediately
            
            # Minimal movement thresholds
            'slam_toolbox.minimum_travel_distance': '0.01',  # Update after tiny moves
            'slam_toolbox.minimum_travel_heading': '0.01',
            
            # Reduce processing load
            'slam_toolbox.resolution': '0.075',  # Even coarser map for speed
            'slam_toolbox.max_iterations': '25',  # Fewer iterations per scan
            
            # Trust sensors more
            'slam_toolbox.position_covariance_scale': '0.1',
            'slam_toolbox.observation_covariance_scale': '0.05',
            }.items()
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}]
        )
    ])