from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Usamos el MISMO nav2_params.yaml que en el ejemplo de simulación
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_params_path = os.path.join(nav2_bringup_share, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 stack'
        ),

        # --- NAV2 NODES (SOLO NAVEGACIÓN, SIN MAP_SERVER NI AMCL) ---

        # Controller server (local planner DWB / FollowPath)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params_path, {
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

        # Global planner
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_path, {
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

        # Behavior server (recovery, spin, back_up, etc.)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params_path, {
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

        # BT Navigator
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params_path, {
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

        # Waypoint follower (opcional, pero no estorba)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[nav2_params_path, {
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),

        # Lifecycle manager SOLO para los nodos de navegación
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower'
                        ]
                    }]
                )
            ]
        ),
    ])
