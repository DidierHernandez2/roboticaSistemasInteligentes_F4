# robotino_bts/behaviors/set_goal_from_location.py

import py_trees
from py_trees.common import Access


class SetGoalFromLocation(py_trees.behaviour.Behaviour):
    """
    Reads pose from node.known_locations[location_name] and writes:
      - goal_x, goal_y, goal_yaw
      - current_location
    to the blackboard.
    """

    def __init__(self, name, node, location_name: str):
        super().__init__(name)
        self.node = node
        self.location_name = location_name

        self.bb = py_trees.blackboard.Client(name=f"SetGoal-{location_name}")
        self.bb.register_key("goal_x", Access.WRITE)
        self.bb.register_key("goal_y", Access.WRITE)
        self.bb.register_key("goal_yaw", Access.WRITE)
        self.bb.register_key("current_location", Access.WRITE)

    def update(self):
        locs = getattr(self.node, "known_locations", None)
        if not isinstance(locs, dict):
            self.node.get_logger().error("[SetGoal] node.known_locations is not a dict")
            return py_trees.common.Status.FAILURE

        if self.location_name not in locs:
            self.node.get_logger().error(f"[SetGoal] Unknown location '{self.location_name}'")
            return py_trees.common.Status.FAILURE

        pose = locs[self.location_name]
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))

        self.bb.goal_x = x
        self.bb.goal_y = y
        self.bb.goal_yaw = yaw
        self.bb.current_location = self.location_name

        self.node.get_logger().info(
            f"[SetGoal] '{self.location_name}': "
            f"goal_x={x:.2f}, goal_y={y:.2f}, yaw={yaw:.2f}"
        )

        return py_trees.common.Status.SUCCESS
