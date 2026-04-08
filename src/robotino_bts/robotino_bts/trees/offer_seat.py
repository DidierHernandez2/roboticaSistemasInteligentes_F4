from platform import node
import py_trees
import numpy as np
from py_trees.common import OneShotPolicy , Access
from robotino_bts.behaviors.init_blackboard_receptionist import InitBlackboard
from robotino_bts.behaviors.navigate_to_known_location import NavToKnownLocation
from robotino_bts.behaviors.yolo_detect import YoloDetectBehaviour
from robotino_bts.behaviors.wait_for_face import FaceRecognitionBehaviour
from robotino_bts.behaviors.wait_for_continue import WaitForContinue
from robotino_bts.behaviors.talk_behaviors import SayTextBehaviour
from robotino_bts.behaviors.utils_receptionist import FreeSeatEquals ,    PersonSeen  , ChooseOnce
import rclpy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time



#def create_behavior_tree(node):
def build_offer_seat_subtree(node):

    """
    Growing working example:
      InitBlackboard -> YoloCall -> WaitForContinue (prints BB contents + pauses)

    You can continue by:
      ros2 topic pub --once /bt/continue std_msgs/msg/Bool "{data: true}"
    """

    node.get_logger().info("[BT] Building RoamKnownLocs tree (INIT + YOLO + WAIT)")

    seq = py_trees.composites.Sequence(
        name="MeetGuestSeq",
        memory=True,
    )
    seq_oneshot = py_trees.decorators.OneShot(
        name="MeetGuestSeqOneshot",
        child=seq,
        policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
    #####################################################
    #init_bb = InitBlackboard()

    #####################################################
    say_look_at_me = SayTextBehaviour(
        name="say look at me ",
        node=node,
        #text="What is your name and what drink would you like?",
        text="I am looking for a place to seat, please look at me",  
        wait=True,
    )
    #####################################################
    yolo_call = YoloDetectBehaviour(
        name="YoloCall",
        node=node,
    )

    
    wait_and_inspect = WaitForContinue(
        name="WAIT_INSPECT_BB",
        continue_key="debug_continue",
        continue_topic="/bt/continue",
        print_every_tick=False,     # print once when we enter the pause
        reset_on_success=True,
        keys_to_print=[
            # YOLO lists
            "yolo_poses_map",
            "free_seat",

            ## anything else you want to see
            #"last_text",
            #"current_guest_name",
            #"current_guest_drink",
        ],
    )
    
    

    person_finder = PersonSeen(node=node)
    wait_and_inspect.setup(node=node)
    
    detect_once= py_trees.composites.Sequence(
        name="DetectOnce",
        memory=False,
    )
    
    detect_once.add_children([
    yolo_call,
    person_finder,
    ])
    ##############################################
    # Retry detecting human until found
    human_retry = py_trees.decorators.Retry(
        name="HumanRetryUntilFound",
        child=detect_once,
        num_failures=10_000,   # effectively "infinite"
    )
    # But cap the total time spent retrying


    human_timeout = py_trees.decorators.Timeout(
        name="HumanTimeout30s",
        child=human_retry,
        duration=30.0,
    )
    
    #detect_person_once = py_trees.decorators.OneShot(
    #name="DetectPersonOnce",
    #child=human_timeout,
    ##policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    #policy= py_trees.common.OneShotPolicy.ON_COMPLETION,
    #)
    
    ###############################################
    present_seat_1 = SayTextBehaviour(
        name="say sit here ",
        node=node,
        #text="What is your name and what drink would you like?",
        text="here is a place to seat",  # callable
        wait=True,
    )
    ###############################################
    present_seat_2 = SayTextBehaviour(
        name="say sit here  2",
        node=node,
        #text="What is your name and what drink would you like?",
        text="here is a place to seat",  # callable
        wait=True,
    )
    
    ###########################################       
    seat_selector = py_trees.composites.Selector(
        name="SeatDecision",
        memory=True,   # stick to chosen branch until it finishes
    )

    #seat_selector=  ChooseOnce(name="SeatDecision")  ### REMOVE FROM UTILS IF DEPERECATED
    case_seat_1 = py_trees.composites.Sequence(name="CaseSeat1", memory=True)
   
    nav_to_seat1 = py_trees.decorators.Retry(
    name="RetryNavToSeat1_5",
    child=NavToKnownLocation(
        name="NavToSeat1",
        node=node,
        location_name="seat_1",
    ),
    num_failures=5,
    )

    case_seat_1.add_children([
        FreeSeatEquals("seat_1"),
        nav_to_seat1,
        present_seat_1,
    ])

    case_seat_2 = py_trees.composites.Sequence(name="CaseSeat2", memory=True)
    nav_to_seat2 = py_trees.decorators.Retry(
    name="RetryNavToSeat2_5",
    child=NavToKnownLocation(
        name="NavToSeat2",
        node=node,
        location_name="seat_2",
    ),
    num_failures=5,
    )
    
    
    
    
    case_seat_2.add_children([
        FreeSeatEquals("seat_2"),
        nav_to_seat2,
        present_seat_2,
    ])

    case_no_seat = py_trees.composites.Sequence(name="CaseNoSeat", memory=True)
    nav_to_place_seat = py_trees.decorators.Retry(
    name="RetryNavToFindseat2_5",
    child=NavToKnownLocation(
        name="NavToFindSeat2",
        node=node,
        location_name="find_seat_2",
    ),
    num_failures=5,
    )
    case_no_seat.add_children([
        FreeSeatEquals("None"),
        nav_to_place_seat,
    ])

    seat_selector.add_children([
        case_seat_1,
        case_seat_2,
        case_no_seat,
    ])
    
    
    ##########################################       
    
    # 
    #  
    seq.add_children([
        #init_bb,
        # goto_door,
        #yolo_call,
        #human_timeout,
        say_look_at_me,
        human_timeout,
        seat_selector,
        wait_and_inspect,
        # face_call,
    ])

   

    return seq_oneshot
