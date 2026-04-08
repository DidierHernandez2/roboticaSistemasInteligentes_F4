# robotino_bts/behaviors/init_blackboard.py

import py_trees
from py_trees.common import Access


class InitBlackboard(py_trees.behaviour.Behaviour):
    """
    Initialize blackboard keys for the BT:
      - target_object: what we want to look for (e.g. "cup")
      - detections_log: list to accumulate YOLO results later
    """

    def __init__(self, name="InitBlackboard", target_object="cup"):
        super().__init__(name)
        self.target_object = target_object

        # Blackboard client
        self.bb = py_trees.blackboard.Client(name="InitBB")
        self.bb.register_key("target_object", Access.WRITE)
        self.bb.register_key("detections_log", Access.WRITE)

    def update(self):
        self.bb.target_object = self.target_object
        self.bb.detections_log = []

        self.logger.info(
            f"Blackboard initialized: target_object='{self.target_object}', "
            f"detections_log=[]"
        )
        return py_trees.common.Status.SUCCESS
