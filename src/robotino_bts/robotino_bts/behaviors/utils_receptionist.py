import py_trees
from py_trees.common import Access
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration
import tf2_ros
import numpy as np  

from rclpy.time import Time

class ChooseOnce(py_trees.composites.Composite):
    """
    Like a selector for choosing a branch, but once a child is selected,
    it NEVER falls through to other children. If the chosen child fails,
    the whole composite fails.
    """
    def __init__(self, name="ChooseOnce", memory=True):
        super().__init__(name)
        self.chosen_index = None
        self.memory = memory

    def tick(self):
        # Choose phase
        if self.chosen_index is None:
            for i, child in enumerate(self.children):
                for node in child.tick():
                    yield node
                status = child.status
                if status in (py_trees.common.Status.RUNNING, py_trees.common.Status.SUCCESS):
                    self.chosen_index = i
                    self.status = status
                    yield self
                    return
                # if FAILURE, try next child during choose phase only

            # none matched
            self.status = py_trees.common.Status.FAILURE
            yield self
            return

        # Commit phase (never try others)
        child = self.children[self.chosen_index]
        for node in child.tick():
            yield node
        self.status = child.status
        yield self
        return

    def stop(self, new_status=py_trees.common.Status.INVALID):
        # reset choice when parent stops us
        self.chosen_index = None
        super().stop(new_status)
###############################################
class PersonSeen(py_trees.behaviour.Behaviour):
    def __init__(self,node, name="PersonSeen"):

        super().__init__(name)
        self.node = node
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key("detections_log", Access.READ)
        self.bb.register_key("yolo_poses_map", Access.WRITE)
        self.bb.register_key("free_seat", Access.WRITE)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)


    def update(self):
        log = getattr(self.bb, "detections_log", [])
        if not log:
            return py_trees.common.Status.RUNNING

        last = log[-1]
        classes = last.get("classes", [])
        poses= last.get("poses", [])

        if "person" not in classes:return py_trees.common.Status.RUNNING#FAILURE
        self.node.get_logger().info("Human detected!")
        self.bb.yolo_poses_map = []

        for cls, pose in zip(classes, poses):
            if cls != "person":
                continue

            x = pose.pose.position.x
            y = pose.pose.position.y
            self.node.get_logger().info(f"Human detected at kinect ({x:.2f}, {y:.2f})")
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
                #############CLEAN THIS
                arr=np.asarray (([-3.7,1.45],[-3.867,2.30])) #seat_1, seat_2
                seats=np.asarray(['seat_1', 'seat_2'])
                self.node.get_logger().info(f"Human in map at ({x:.2f}, {y:.2f})")
                human_pose=np.asarray((x,y))
                dist = np.linalg.norm(arr - human_pose, axis=1)
                occupied_mask = dist < 0.50
                occupied_seats = seats[occupied_mask]
                free_seats = seats[~occupied_mask]                    
                self.node.get_logger().info(f"Occupied seat: {occupied_seats}")
                self.node.get_logger().info(f"Free seat: {free_seats[0]}")
                ###########################
                

                self.bb.yolo_poses_map.append((x, y))
                self.bb.free_seat = free_seats[0]


            except Exception as e:
                self.node.get_logger().warn(f"[BT] TF to map failed: {e}")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

###############################################
class FreeSeatEquals(py_trees.behaviour.Behaviour):
    def __init__(self, expected_value: str):
        super().__init__(name=f"FreeSeat == '{expected_value}'")
        self.expected_value = expected_value
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("free_seat", Access.READ)


    def update(self):
        value = self.bb.free_seat
        if value == self.expected_value:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE