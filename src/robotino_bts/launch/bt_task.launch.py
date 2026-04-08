from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

import os
from pathlib import Path
import shutil


def _resolve_known_locations_yaml(context, *args, **kwargs):
    """
    Resolve a portable known_locations.yaml path:

      - Default runtime path: ~/.robotino/known_locations.yaml
      - If it doesn't exist, seed it from robotino_bts packaged default.
      - If user passes known_locations_file, use that (must exist).
    """
    user_override = LaunchConfiguration("known_locations_file").perform(context)

    # 1) User override (explicit path)
    if user_override:
        p = Path(os.path.expanduser(user_override))
        if not p.is_absolute():
            # relative -> relative to current working directory of launch
            p = (Path.cwd() / p).resolve()
        if not p.exists():
            raise RuntimeError(f"[launch] known_locations_file does not exist: {p}")
        yaml_path = str(p)
        return [
            Node(
                package="robotino_bts",
                executable="task_manager",
                name="bt_executor",
                output="screen",
                parameters=[{"known_locations": yaml_path}],
            ),
            Node(
                package="known_locations_tf_server",
                executable="known_locations_server",
                name="known_locations_server",
                output="screen",
                parameters=[{"locations_file": yaml_path}],
            ),
             Node(
                package="robot_movement",
                executable="pid_yaw",
                name="pid_yaw",
                output="screen",
                
            ),
        ]

    # 2) Default user path
    user_dir = Path.home() / ".robotino"
    user_dir.mkdir(parents=True, exist_ok=True)
    user_yaml = user_dir / "known_locations.yaml"

    # Seed from packaged default if missing
    if not user_yaml.exists():
        print (f"[launch] known_locations.yaml not found at {user_yaml}, ")

    yaml_path = str(user_yaml)

    return [
    Node(
        package="robotino_bts",
        executable="task_manager",
        name="bt_executor",
        output="screen",
        parameters=[{"known_locations": yaml_path}],
    ),
    Node(
        package="known_locations_tf_server",
        executable="known_locations_server",
        name="known_locations_server",
        output="screen",
        parameters=[{"locations_file": yaml_path}],
    ),
    Node(
        package="robot_movement",
        executable="pid_yaw",
        name="pid_yaw",
        output="screen",
    ),    ]



def generate_launch_description():
    with_viewer = LaunchConfiguration("with_viewer")

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_viewer",
            default_value="false",
            description="Start py_trees_ros_viewer GUI (py-trees-tree-viewer)"
        ),
        DeclareLaunchArgument(
            "known_locations_file",
            default_value="",
            description="Optional absolute path to known_locations.yaml. If empty, uses ~/.robotino/known_locations.yaml"
        ),

        # Create the two nodes with a resolved shared YAML path
        OpaqueFunction(function=_resolve_known_locations_yaml),

        # Optional PyTrees GUI Viewer
        ExecuteProcess(
            condition=IfCondition(with_viewer),
            cmd=["py-trees-tree-viewer"],
            output="screen",
        ),
    ])
