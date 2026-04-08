from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # TF: odom -> base_footprint  
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),

        # TF: base_footprint -> laser  (THE MOTHERFUCKING MISSING ONE)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            name='base_footprint_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'laser']
        ),
    ])