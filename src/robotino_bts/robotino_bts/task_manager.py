#!/usr/bin/env python3
# robotino_bts/task_manager.py

import os
import time
from pathlib import Path
import shutil

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import yaml

import py_trees
import py_trees_ros
#
#from ament_index_python.packages import get_package_share_directory
from robotino_bts.trees.receptionist import create_behavior_tree

#from robotino_bts.trees.offer_seat import create_behavior_tree
#from robotino_bts.trees.meet_guest import create_behavior_tree


class BTExecutor(Node):
    """
    Owns known_locations and builds the BT root. The py_trees_ros BehaviourTree
    will create its own internal node named /tree for introspection.
    """

    def __init__(self):
        super().__init__("bt_executor_main")

        # ------------------------------------------------------------------ #
        # 1) Declare + resolve known_locations YAML path (portable & editable)
        # ------------------------------------------------------------------ #
        self.declare_parameter("known_locations", "")
        param_path = str(self.get_parameter("known_locations").value).strip()

        known_locations_file = self._resolve_known_locations_file(param_path)

        # ------------------------------------------------------------------ #
        # 2) Load known locations into self.known_locations
        # ------------------------------------------------------------------ #
        try:
            self.get_logger().info(f"[bt_executor] Loading known locations from: {known_locations_file}")

            with open(known_locations_file, "r") as f:
                data = yaml.safe_load(f) or {}

            self.known_locations = data if isinstance(data, dict) else {}
            self.get_logger().info(f"[bt_executor] Loaded known locations: {list(self.known_locations.keys())}")
        except Exception as e:
            self.get_logger().error(f"[bt_executor] Failed to load known locations file: {e}. Using empty dict.")
            self.known_locations = {}

        # ------------------------------------------------------------------ #
        # 3) Build tree + setup introspection node
        # ------------------------------------------------------------------ #
        built = create_behavior_tree(self)
        root = built.root
        self.tree = py_trees_ros.trees.BehaviourTree(root)
        try:
            self.tree.setup(node=None, node_name="tree", timeout=15.0)
        except Exception as e:
            self.get_logger().error(f"[bt_executor] Error in tree.setup(): {e}")

        self.get_logger().info("[bt_executor] Behavior Tree setup complete (introspection node: /tree)")

    def _resolve_known_locations_file(self, override: str) -> str:
        """
        Resolve known locations yaml path.

        Priority:
          1) override from ROS param 'known_locations' (must exist)
          2) ~/.robotino/known_locations.yaml (seeded if missing)
        """
        # 1) Explicit override
        if override:
            p = Path(os.path.expanduser(override))
            p = p if p.is_absolute() else (Path.cwd() / p).resolve()
            if not p.exists():
                raise FileNotFoundError(f"known_locations param points to missing file: {p}")
            return str(p)

        # 2) Default user path (ROS1-style)
        user_dir = Path.home() / ".robotino"
        user_dir.mkdir(parents=True, exist_ok=True)
        user_yaml = user_dir / "known_locations.yaml"

        # Seed if missing (prefer workspace src, fallback to package share)
        if not user_yaml.exists():
            ws_src_yaml = Path.home() / "robotino_ros2_ws" / "src" / "robotino_bts" / "config" / "known_locations.yaml"
            if ws_src_yaml.exists():
                seed_from = ws_src_yaml
            else:
                pkg_share = Path(get_package_share_directory("robotino_bts"))
                seed_from = pkg_share / "config" / "known_locations.yaml"

            if not seed_from.exists():
                raise FileNotFoundError(f"No seed YAML found at: {seed_from}")

            shutil.copyfile(seed_from, user_yaml)
            self.get_logger().info(f"[bt_executor] Seeded user known locations: {user_yaml}")

        return str(user_yaml)

    def tick_tree(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"[bt_executor] Exception during BT tick: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = BTExecutor()
    node.get_logger().info("[bt_executor] Starting Behavior Tree...")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # IMPORTANT: also spin the internal py_trees_ros node (/tree)
    if getattr(node.tree, "node", None) is not None:
        executor.add_node(node.tree.node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            node.tick_tree()
            time.sleep(0.05)
    except KeyboardInterrupt:
        node.get_logger().info("[bt_executor] KeyboardInterrupt, shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
