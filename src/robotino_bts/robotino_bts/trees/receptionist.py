import py_trees
from py_trees.common import OneShotPolicy

from robotino_bts.behaviors.init_blackboard_receptionist import InitBlackboard
from robotino_bts.trees.meet_guest import build_meet_guest_subtree
from robotino_bts.trees.offer_seat import build_offer_seat_subtree
####################3
from robotino_bts.behaviors.navigate_to_known_location import NavToKnownLocation

def create_behavior_tree(node):
    seq = py_trees.composites.Sequence(
        name="Receptionist",
        memory=True,
    )

    init_bb = InitBlackboard(host="jack")
    nav_to_place_seat = py_trees.decorators.Retry(
    name="RetryNavToFindseat1_5",
    child=NavToKnownLocation(
        name="NavToFindSeat1",
        node=node,
        location_name="find_seat_1",
    ),
    num_failures=5,
    )
    seq.add_children([
        init_bb,
        build_meet_guest_subtree(node),
        nav_to_place_seat,
        build_offer_seat_subtree(node),
    ])

    root = py_trees.decorators.OneShot(
        name="ROOT",
        child=seq,
        policy=OneShotPolicy.ON_COMPLETION,
    )

    return py_trees.trees.BehaviourTree(root)
