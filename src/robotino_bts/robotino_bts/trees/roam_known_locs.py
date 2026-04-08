# robotino_bts/trees/roam_known_locs.py

import py_trees
import numpy as np
from py_trees.common import OneShotPolicy , Access
from robotino_bts.behaviors.init_blackboard_receptionist import InitBlackboard
from robotino_bts.behaviors.navigate_to_known_location import NavToKnownLocation
from robotino_bts.behaviors.yolo_detect import YoloDetectBehaviour
from robotino_bts.behaviors.wait_for_face import FaceRecognitionBehaviour
from robotino_bts.behaviors.wait_for_continue import WaitForContinue
import rclpy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time



def create_behavior_tree(node):
    """
    Growing working example:
      InitBlackboard -> YoloCall -> WaitForContinue (prints BB contents + pauses)

    You can continue by:
      ros2 topic pub --once /bt/continue std_msgs/msg/Bool "{data: true}"
    """

    node.get_logger().info("[BT] Building RoamKnownLocs tree (INIT + YOLO + WAIT)")

    seq = py_trees.composites.Sequence(
        name="ReceptionistSeq",
        memory=True,
    )

    #####################################################
    init_bb = InitBlackboard()

    #####################################################
    goto_door = NavToKnownLocation(
        name="NavToDoor",
        node=node,
        location_name="living_room",
    )

    #####################################################
    yolo_call = YoloDetectBehaviour(
        name="YoloCall",
        node=node,
    )

    #####################################################
    # NEW: pause + dump blackboard keys we care about
    #
    # Assumes your YOLO behavior writes these keys:
    #   - yolo_class_names: list[str]
    #   - yolo_poses_map  : list[PoseStamped]
    #   - yolo_best_name  : str (optional)
    #   - yolo_best_pose  : PoseStamped (optional)
    #
    # If your key names differ, just change this list to match.
    wait_and_inspect = WaitForContinue(
        name="WAIT_INSPECT_BB",
        continue_key="debug_continue",
        continue_topic="/bt/continue",
        print_every_tick=False,     # print once when we enter the pause
        reset_on_success=True,
        keys_to_print=[
            # YOLO lists
            "yolo_class_names",
            "yolo_poses_map",

            # single best pick (if you store it)
            "yolo_best_name",
            "yolo_best_pose",

            # anything else you want to see
            "last_text",
            "current_guest_name",
            "current_guest_drink",
        ],
    )
    
    class PersonSeen(py_trees.behaviour.Behaviour):
        def __init__(self,node, name="PersonSeen"):

            super().__init__(name)
            self.node = node
            self.bb = self.attach_blackboard_client(name=name)
            self.bb.register_key("detections_log", Access.READ)
            self.bb.register_key("yolo_poses_map", Access.WRITE)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)


        def update(self):
            log = getattr(self.bb, "detections_log", [])
            if not log:
                return py_trees.common.Status.FAILURE

            last = log[-1]
            classes = last.get("classes", [])
            poses= last.get("poses", [])

            if "person" not in classes:return py_trees.common.Status.FAILURE
            node.get_logger().info("Human detected!")
            self.bb.yolo_poses_map = []

            for cls, pose in zip(classes, poses):
                if cls != "person":
                    continue

                x = pose.pose.position.x
                y = pose.pose.position.y
                node.get_logger().info(f"Human detected at kinect ({x:.2f}, {y:.2f})")
                #pose.header.stamp = Time().to_msg()   # latest available TF time
                pose.header.frame_id = "kinect_link"
                
                # FORCE latest: zero stamp
                pose.header.stamp.sec = 0
                pose.header.stamp.nanosec = 0

                t = Time()
                if not self.tf_buffer.can_transform("map", "kinect_link", Time(), timeout=Duration(seconds=0.2)):
                    self.node.get_logger().info("[BT] Waiting for TF map<-kinect_link...")
                    return py_trees.common.Status.RUNNING
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "map",               # target
                        "kinect_link",       # source
                        t,
                        timeout=Duration(seconds=0.4),
                    )

                    pose_map = do_transform_pose_stamped(pose, tf)
                    pose_map.header.frame_id = "map"

                    x = pose_map.pose.position.x
                    y = pose_map.pose.position.y

                    node.get_logger().info(f"Human in map at ({x:.2f}, {y:.2f})")

                    self.bb.yolo_poses_map.append((x, y))

                except Exception as e:
                    node.get_logger().warn(f"[BT] TF to map failed: {e}")
                    return py_trees.common.Status.RUNNING


            return py_trees.common.Status.SUCCESS
            
        
            
            


    person_finder = PersonSeen(node=node)
    wait_and_inspect.setup(node=node)

    #####################################################
    face_call = FaceRecognitionBehaviour(
        name="FaceCall",
        node=node,
    )

    seq.add_children([
        init_bb,
        # goto_door,
        yolo_call,
        person_finder,
        wait_and_inspect,
        # face_call,
    ])

    root = py_trees.decorators.OneShot(
        name="ROOT",
        child=seq,
        policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )

    return py_trees.trees.BehaviourTree(root)
