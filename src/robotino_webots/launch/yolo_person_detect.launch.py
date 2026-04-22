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

    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = os.path.join(robotino_webots_share, 'urdf', 'Robotino3.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    nav2_params_path = os.path.join(robotino_webots_share, 'config', 'nav2_robotino_webots_pure_pursuit.yaml')

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
            'map_file',
            default_value=os.path.expanduser('~/robotec_ws/map_2.yaml'),
            description='Path to map YAML file'
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

        # 4. Static transform: map -> odom (bootstrap mientras slam_toolbox establece el frame)
        TimerAction(period=7.0, actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            )
        ]),

        # 5. Map server
        TimerAction(period=8.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file,
                    'frame_id': 'map'
                }]
            )
        ]),

        # 6. AMCL — disabled: slam_toolbox handles map->odom TF during mapping
        # TimerAction(period=10.0, actions=[
        #     Node(package='nav2_amcl', executable='amcl', name='amcl', ...)
        # ]),

        # 7. Controller server
        TimerAction(period=12.0, actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            )
        ]),

        # 8. Planner server
        TimerAction(period=14.0, actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time,
                    'local_costmap.obstacle_layer.observation_persistence': 0.0}]
            )
        ]),

        # 8.5 Smoother server
        TimerAction(period=15.0, actions=[
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
            )
        ]),

        # 9. Behavior server
        TimerAction(period=16.0, actions=[
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]),

        # 10. Velocity smoother
        TimerAction(period=17.0, actions=[
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
                    ('odom', 'odom'),
                ]
            )
        ]),

        # 11. Collision monitor
        TimerAction(period=18.0, actions=[
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
                    ('cmd_vel', 'cmd_vel'),
                    ('scan', 'scan'),
                ]
            )
        ]),

        # 12. BT Navigator
        TimerAction(period=18.0, actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
            )
        ]),

        # 13a. Lifecycle manager dedicado para slam_toolbox (arranca antes que Nav2)
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

        # 13b. Lifecycle manager para Nav2
        TimerAction(period=20.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'map_server',
                        'controller_server',
                        'planner_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'velocity_smoother',
                        'collision_monitor',
                    ]
                }]
            )
        ]),

        # TF bridges (from robotino.launch.py)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='kinect_depth_tf_glue',
            arguments=['0', '0', '0', '0', '0', '0', 'kinect_link', 'kinect_depth'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        # 14. slam_toolbox — arranca temprano para que el frame 'map' exista
        #     antes de que Nav2 (controller, planner, etc.) lo necesite
        TimerAction(period=7.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(robotino_webots_share, 'config', 'slam_toolbox.yaml'),
                    {'use_sim_time': use_sim_time,
                     'scan_topic':   '/scan',
                     'odom_frame':   'odom',
                     'map_frame':    'map',
                     'base_frame':   'base_link'}
                ]
            )
        ]),

        # 15. RViz con config de mapeo (Map + LaserScan + RobotModel)
        TimerAction(period=24.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(robotino_webots_share, 'config', 'mapping.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]),

        # --- Path planning node (A* sobre mapa de slam_toolbox) ---
        TimerAction(period=23.0, actions=[
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
            ),
            Node(
                package='vision',
                executable='potential_field_viz_node',
                name='potential_field_viz_node',
                output='screen',
                parameters=[{
                    'k_att':       1.0,
                    'k_rep':       3.0,
                    'influence_m': 1.5,
                    'update_hz':   1.0,
                }]
            ),
        ]),

        # --- Map nodes (trace + bayesian + laser accumulator) ---
        TimerAction(period=24.5, actions=[
            Node(
                package='vision',
                executable='map_trace_node',
                name='map_trace_node',
                output='screen',
            ),
            Node(
                package='vision',
                executable='bayesian_mapper_node',
                name='bayesian_mapper_node',
                output='screen',
                parameters=[{
                    'resolution':    0.10,
                    'map_width_m':  30.0,
                    'map_height_m': 30.0,
                    'publish_hz':    2.0,
                    'l_occ':         0.85,
                    'l_free':       -0.40,
                    'l_min':        -5.0,
                    'l_max':         5.0,
                }]
            ),
            Node(
                package='vision',
                executable='laser_map_node',
                name='laser_map_node',
                output='screen',
                parameters=[{
                    'quantize_m': 0.08,
                    'publish_hz': 2.0,
                }]
            ),
        ]),

        # --- Vision nodes (yolo + obstacle avoidance) ---
        TimerAction(period=25.0, actions=[
            Node(
                package='vision',
                executable='yolo_person_node',
                name='yolo_person_node',
                output='screen',
                parameters=[{
                    'image_topic':    '/kinect_sim/rgb/image_raw',
                    'depth_topic':    '/kinect_sim/depth/image_raw',
                    'model_path':     'yolo11n-seg.pt',
                    'search_ang_vel': 0.1,
                }]
            ),
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
            ),
        ]),
    ])
