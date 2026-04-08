# robotino_bts/trees/meet_guest.py
#
# Full drop-in file (NO dummy SayText).
# Uses your real TTS service:
#   Service: /tts/talk
#   Type:    robotino_interfaces/srv/Talk
#
# Behavior notes:
# - SayTextBehaviour sends a service request and returns:
#     RUNNING while waiting for the service reply
#     SUCCESS when reply success=True
#     FAILURE otherwise
# - If you pass wait=True in the request, the TTS server blocks until finished speaking.
#   So this BT node effectively becomes "sync speech" (sequence-friendly).

import time
import py_trees
from py_trees.common import OneShotPolicy, ComparisonExpression

from robotino_interfaces.srv import Talk
from robotino_bts.behaviors.wait_for_continue import WaitForContinue
from robotino_bts.behaviors.init_blackboard_receptionist import InitBlackboard
from robotino_bts.behaviors.navigate_to_known_location import NavToKnownLocation
from robotino_bts.behaviors.wait_for_face import FaceRecognitionBehaviour
from robotino_bts.behaviors.train_face import TrainFace
from robotino_bts.behaviors.wait_for_text import WaitForText
from robotino_bts.behaviors.parse_receptionist import ParseGuestFromText
from robotino_bts.behaviors.talk_behaviors import SayTextBehaviour
from robotino_bts.behaviors.set_grammar_mode import SetGrammarMode

def outro_text(node):
    bb = py_trees.blackboard.Client(name="OutroText")
    bb.register_key("current_guest_name", py_trees.common.Access.READ)
    name = (getattr(bb, "current_guest_name", "") or "").strip()
    node.get_logger().info(f"[OUTRO_TEXT] current_guest_name='{name}'")

    if name:
        return f"Nice to meet you {name}. Please follow me."
    return "Nice to meet you. Please follow me."


def ask_name_drink_text(node):
    bb = py_trees.blackboard.Client(name="AskDrinkText")
    bb.register_key("current_guest_name", py_trees.common.Access.READ)
    bb.register_key("current_guest_drink", py_trees.common.Access.READ)
    name = (getattr(bb, "current_guest_name", "") or "").strip()
    drink = (getattr(bb, "current_guest_drink", "") or "").strip()
    node.get_logger().info(f"[ASK_TEXT] name='{name}' drink='{drink}'")
    if name and drink:
        return ""
    if name and name != "unknown":
        return f"Hi {name}. What drink would you like?"
    if drink:
        return f"I heard {drink}. Please repeat your name."
    return "What is your name and what drink would you like?"

class WaitSeconds(py_trees.behaviour.Behaviour):
    def __init__(self, name, seconds: float):
        super().__init__(name)
        self.seconds = seconds
        self._t0 = None

    def initialise(self):
        self._t0 = time.time()

    def update(self):
        return (py_trees.common.Status.SUCCESS
                if (time.time() - self._t0) >= self.seconds
                else py_trees.common.Status.RUNNING)

#####################################################START OF TREE DEFINITION

#def create_behavior_tree(node):

def build_meet_guest_subtree(node):

    seq = py_trees.composites.Sequence(
        name="Meet Guest",
        memory=True
    )

    # -------------------------------------------------
    # Init + intro + navigate
    init_bb = InitBlackboard(host="jack")
    ######################################
    known_names = ["jack", "oscar", "maria", "john"]
    known_drinks = ["water", "coke", "tea", "coffee", "juice", "beer"]
    connectors = [
        "my name is",
        "i am",
        "i would like",
        "i want",
        "and",
    ]

    grammar_phrases = known_names + known_drinks + connectors
    #########################
    set_custom_grammar = SetGrammarMode(
        name="GrammarNameDrinkCustom",
        node=node,
        mode="CUSTOM",
        phrases=grammar_phrases,
    )
    intro = SayTextBehaviour(
        name="IntroTalk",
        node=node,
        text="Hello. I will go to the door.",
        wait=True,
    )
    
    
    
    pre_door = py_trees.composites.Sequence(name="PreDoor", memory=True)
    pre_door.add_children([intro, set_custom_grammar])

    pre_door_oneshot = py_trees.decorators.OneShot(
        name="PreDoorOneShot",
        child=pre_door,
        policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )

    
    
    goto_door = py_trees.decorators.Retry(
    name="RetryNavDoor_5",
    child=NavToKnownLocation(
        name="NavDoor",
        node=node,
        location_name="door",
    ),
    num_failures=5,
    )
    
   
    greet = SayTextBehaviour(
        name="IntroTalk",
        node=node,
        text="Hello .Please look at me.",
        wait=True,
    )
    # -------------------------------------------------
    # FACE BLOCK (<= 30s): keep retrying until a face is found
    face_try = FaceRecognitionBehaviour(
        name="AcquireFace",
        node=node,
    )

    # Retry indefinitely (or “a lot”), until SUCCESS
    #face_retry = py_trees.decorators.Retry(
    #    name="FaceRetryUntilFound",
    #    child=face_try,
    #    num_failures=10_000,   # effectively "infinite"
    #)

    # But cap the total time spent retrying
    face_timeout = py_trees.decorators.Timeout(
        name="FaceTimeout30s",
        child=face_try,
        duration=30.0,
    )


    # -------------------------------------------------
    # SPEECH BLOCK (retry max 5)
    ask = SayTextBehaviour(
        name="AskNameDrink",
        node=node,
        #text="What is your name and what drink would you like?",
        text=ask_name_drink_text,  # callable
        wait=True,
    )
    cooldown = WaitSeconds("AfterAskCooldown", 1.8)   
    
    

    listen = WaitForText(
        name="ListenText",
        node=node,
    )



    parse = ParseGuestFromText(
        name="ParseNameDrink",
        known_names=known_names,
        known_drinks=known_drinks,
        text_key="speech_last_text",
    )

    ask_and_listen = py_trees.composites.Sequence(name="AskListenParse", memory=True)
    ask_and_listen.add_children([ask,cooldown, listen, parse])

    speech_retry = py_trees.decorators.Retry(
        name="SpeechRetry5Times",
        child=ask_and_listen,
        num_failures=5,
    )

   
    face_train = TrainFace(
        name="TrainFace",
        node=node,
        name_key="current_guest_name",
        ) 

    outro = SayTextBehaviour(
        name="OutroTalk",
        node=node,
        text=outro_text,  # callable
        wait=True,
    )



    set_free_grammar = SetGrammarMode(
        name="GrammarFree",
        node=node,
        mode="FREE",
        phrases=[], 
        
    )

    wait_and_inspect = WaitForContinue(
        name="WAIT_INSPECT_BB",
        continue_key="debug_continue",
        continue_topic="/bt/continue",
        print_every_tick=False,     # print once when we enter the pause
        reset_on_success=True,
        keys_to_print=[
            # YOLO lists


        ],
    )
    wait_and_inspect.setup(node=node)

    # -------------------------------------------------
    seq.add_children([
        #init_bb,
        pre_door_oneshot,
        goto_door,
        greet,
        face_timeout,
        speech_retry,
        #wait_and_inspect,
        face_train,
        outro,
        set_free_grammar,
    ])
    

   
    seq_oneshot = py_trees.decorators.OneShot(
        name="MeetGuestSeqOneshot",
        child=seq,
        policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
    return seq_oneshot
    #return seq
    