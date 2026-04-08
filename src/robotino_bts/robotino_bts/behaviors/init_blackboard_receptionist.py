import py_trees
from py_trees.common import Access


class InitBlackboard(py_trees.behaviour.Behaviour):
    """
    Receptionist blackboard initializer.

    keys:
      - host
      - guest_1, guest_2
      - detections_log
      - current_guest_label, current_guest_name, current_guest_drink
      - face_ok, face_names, needs_training
      - speech_last_text, speech_last_stamp_ns
      - guest_counter
    """

    def __init__(self, name="InitBlackboard", host="jack"):
        super().__init__(name)
        self.host = host

        self.bb = py_trees.blackboard.Client(name="InitBB")

        # Existing keys
        self.bb.register_key("host", Access.WRITE)
        self.bb.register_key("guest_1", Access.WRITE)
        self.bb.register_key("guest_2", Access.WRITE)
        self.bb.register_key("detections_log", Access.WRITE)

        # New receptionist keys (flat for now; we can namespace later)
        self.bb.register_key("current_guest_label", Access.WRITE)
        self.bb.register_key("current_guest_name", Access.WRITE)
        self.bb.register_key("current_guest_drink", Access.WRITE)

        self.bb.register_key("face_ok", Access.WRITE)
        self.bb.register_key("face_names", Access.WRITE)
        self.bb.register_key("needs_training", Access.WRITE)

        self.bb.register_key("speech_last_text", Access.WRITE)
        self.bb.register_key("speech_last_stamp_ns", Access.WRITE)

        self.bb.register_key("guest_counter", Access.WRITE)
        
        self.bb.register_key("free_seat", Access.WRITE)


    def initialise(self):
        

        self.bb.host = self.host
        self.bb.guest_1 = []
        self.bb.guest_2 = []
        self.bb.detections_log = []

        self.bb.current_guest_label = ""
        self.bb.current_guest_name = ""
        self.bb.current_guest_drink = ""

        self.bb.face_ok = False
        self.bb.face_names = []
        self.bb.needs_training = False

        self.bb.speech_last_text = ""
        self.bb.speech_last_stamp_ns = 0
        # If you always want fresh numbering each run, set = 0 unconditionally.
        self.bb.guest_counter = 0
        
        self.bb.free_seat = ""

        self.logger.info(
            f"[InitBB] host='{self.host}', guest_1=[], guest_2=[], detections_log=[], "
            f"current_guest_name='', current_guest_drink='', face_ok=False"
        )

    def update(self):
        return py_trees.common.Status.SUCCESS
