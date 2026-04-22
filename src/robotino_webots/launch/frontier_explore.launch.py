from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

WEBOTS_CONTROLLER_LIB = '/snap/webots/current/usr/share/webots/lib/controller/python'
WEBOTS_HOME = '/snap/webots/current/usr/share/webots'


def generate_launch_description():
    robotino_webots_share = get_package_share_directory('robotino_webots')

    world_file   = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = os.path.join(robotino_webots_share, 'urdf', 'Robotino3.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        SetEnvironmentVariable(name='WEBOTS_HOME', value=WEBOTS_HOME),
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=WEBOTS_CONTROLLER_LIB + ':' + os.environ.get('PYTHONPATH', '')
        ),

        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(robotino_webots_share, 'worlds', 'robotino_apartment.wbt'),
            description='Path to Webots world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 1. Webots
        ExecuteProcess(cmd=['webots', world_file], output='screen'),

        # 2. Robot controller
        TimerAction(period=5.0, actions=[
            Node(
                package='robotino_webots',
                executable='robotino_webots_controller.py',
                name='robotino_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]),

        # 3. Robot state publisher
        TimerAction(period=6.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 30.0
                }]
            )
        ]),

        # 4. TF estáticos
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='kinect_depth_tf_glue',
            arguments=['0', '0', '0', '0', '0', '0', 'kinect_link', 'kinect_depth'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),

        # 5. Static map → odom (bootstrap hasta que slam_toolbox lo tome)
        TimerAction(period=7.0, actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            )
        ]),

        # 6. slam_toolbox — construye el mapa mientras el robot explora
        TimerAction(period=7.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(robotino_webots_share, 'config', 'slam_toolbox.yaml'),
                    {'use_sim_time': use_sim_time,
                     'scan_topic':  '/scan',
                     'odom_frame':  'odom',
                     'map_frame':   'map',
                     'base_frame':  'base_link'}
                ]
            )
        ]),

        # 7. Lifecycle manager para slam_toolbox
        TimerAction(period=9.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                }]
            )
        ]),

        # 8. path_planning_node — A* sobre el mapa de slam_toolbox
        TimerAction(period=12.0, actions=[
            Node(
                package='vision',
                executable='path_planning_node',
                name='path_planning_node',
                output='screen',
                parameters=[{
                    'inflation_radius': 4,
                    'waypoint_step':    8,
                    'map_topic':        '/map',
                }]
            )
        ]),

        # 9. obstacle_avoidance_node — filtra /cmd_vel_desired → /cmd_vel
        TimerAction(period=13.0, actions=[
            Node(
                package='vision',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[{
                    'laser_topic':     '/scan',
                    'depth_topic':     '/kinect_sim/depth/image_raw',
                    'cmd_vel_in':      '/cmd_vel_desired',
                    'cmd_vel_out':     '/cmd_vel',
                    'depth_row_min':   150,
                    'depth_row_max':   330,
                    'lidar_max_range': 1.5,
                    'depth_max_range': 1.5,
                    'rep_min_mag':     80.0,
                    'rep_lin_gain':    0.002,
                    'rep_ang_gain':    0.002,
                    'max_ang_corr':    0.3,
                }]
            )
        ]),

        # 10. laser_map_node — puntos LIDAR persistentes en RViz
        TimerAction(period=13.0, actions=[
            Node(
                package='vision',
                executable='laser_map_node',
                name='laser_map_node',
                output='screen',
                parameters=[{
                    'quantize_m': 0.08,
                    'publish_hz': 2.0,
                }]
            )
        ]),

        # 11. frontier_exploration_node — exploración autónoma
        TimerAction(period=14.0, actions=[
            Node(
                package='vision',
                executable='frontier_exploration_node',
                name='frontier_exploration_node',
                output='screen',
                parameters=[{
                    'spin_ang_vel':    0.15,
                    'min_cluster':     8,
                    'replan_interval': 5,
                }]
            )
        ]),

        # 12. RViz
        TimerAction(period=15.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(robotino_webots_share, 'config', 'frontier.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]),
    ])
