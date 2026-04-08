#!/usr/bin/env python3
# robotino_audio/vosk_node.py

import json
import os
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from vosk import Model, KaldiRecognizer

# ---- add your service import ----
# adjust package name to wherever you place the srv
from robotino_interfaces.srv import SetGrammar


class VoskNode(Node):
    def __init__(self):
        super().__init__("vosk_stt_node")

        # -------------------- Parameters --------------------
        self.declare_parameter("audio_topic", "/audio")
        self.declare_parameter("model_path", "/home/oscar/.cache/vosk/vosk-model-small-en-us-0.15")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("publish_partial", True)

        # Optional: start mode
        self.declare_parameter("grammar_mode", "FREE")  # FREE | NAMES | DRINKS | ...

        # Optional: baked-in phrase lists (you can also load from yaml later)
        self.declare_parameter("names", [])       # string[]
        self.declare_parameter("drinks", [])      # string[]
        self.declare_parameter("locations", [])   # string[]
        self.declare_parameter("yesno", ["yes", "no"])  # default

        self.audio_topic = self.get_parameter("audio_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.publish_partial = bool(self.get_parameter("publish_partial").value)

        # Basic validation
        if not self.model_path:
            raise RuntimeError("Parameter 'model_path' is empty.")
        if not os.path.isdir(self.model_path):
            raise RuntimeError(f"Vosk model_path does not exist or is not a directory: {self.model_path}")
        if self.sample_rate <= 0:
            raise RuntimeError(f"Invalid sample_rate: {self.sample_rate}")

        self.get_logger().info(f"[vosk] audio_topic: {self.audio_topic}")
        self.get_logger().info(f"[vosk] sample_rate: {self.sample_rate}")
        self.get_logger().info(f"[vosk] model_path: {self.model_path}")
        self.get_logger().info(f"[vosk] publish_partial: {self.publish_partial}")

        # -------------------- Vosk init --------------------
        self.model = Model(self.model_path)

        # Lock guarding recognizer swaps + use
        self._rec_lock = threading.Lock()

        # Phrase banks (normalized to lowercase)
        self._phrase_banks = {
            "NAMES": [s.lower() for s in self.get_parameter("names").value],
            "DRINKS": [s.lower() for s in self.get_parameter("drinks").value],
            "LOCATIONS": [s.lower() for s in self.get_parameter("locations").value],
            "YESNO": [s.lower() for s in self.get_parameter("yesno").value],
        }

        # Active mode
        self._grammar_mode = str(self.get_parameter("grammar_mode").value).upper().strip() or "FREE"

        # Build recognizer with the selected mode
        self.rec = None
        self._rebuild_recognizer(self._grammar_mode, custom_phrases=None)

        # -------------------- ROS I/O --------------------
        self.pub_text = self.create_publisher(String, "/speech/text", 10)
        self.pub_partial = self.create_publisher(String, "/speech/partial", 10)

        self.sub_audio = self.create_subscription(
            AudioData,
            self.audio_topic,
            self._audio_cb,
            50,
        )

        # Service: switch grammar mode at runtime
        self.srv_set_grammar = self.create_service(
            SetGrammar,
            "set_grammar_mode",
            self._handle_set_grammar_mode
        )

        self.get_logger().info(f"[vosk] Ready. grammar_mode={self._grammar_mode}")

        self._last_partial = ""

    # -------------------- Grammar switching --------------------
    def _rebuild_recognizer(self, mode: str, custom_phrases=None) -> bool:
        """
        Rebuild self.rec according to mode.
        mode:
          FREE -> no grammar
          NAMES/DRINKS/LOCATIONS/YESNO -> phrase list grammar
          CUSTOM -> use custom_phrases (list of strings)
        """
        mode = (mode or "FREE").upper().strip()

        if mode == "FREE":
            new_rec = KaldiRecognizer(self.model, self.sample_rate)
        elif mode == "CUSTOM":
            phrases = [p.lower().strip() for p in (custom_phrases or []) if p and p.strip()]
            if not phrases:
                self.get_logger().warn("[vosk] CUSTOM mode requested but phrases list is empty; falling back to FREE.")
                new_rec = KaldiRecognizer(self.model, self.sample_rate)
                mode = "FREE"
            else:
                new_rec = KaldiRecognizer(self.model, self.sample_rate, json.dumps(phrases))
        else:
            phrases = self._phrase_banks.get(mode, [])
            if not phrases:
                self.get_logger().warn(f"[vosk] Mode '{mode}' has empty phrase list; falling back to FREE.")
                new_rec = KaldiRecognizer(self.model, self.sample_rate)
                mode = "FREE"
            else:
                new_rec = KaldiRecognizer(self.model, self.sample_rate, json.dumps(phrases))

        new_rec.SetWords(True)
        # Optional: alternatives can help you choose later
        # new_rec.SetMaxAlternatives(3)

        with self._rec_lock:
            self.rec = new_rec
            self._grammar_mode = mode
            self._last_partial = ""  # reset partial spam guard whenever we swap

        self.get_logger().info(f"[vosk] Recognizer rebuilt. grammar_mode={self._grammar_mode}")
        return True

    def _handle_set_grammar_mode(self, req: SetGrammar.Request, resp: SetGrammar.Response):
        mode = (req.mode or "").upper().strip()
        if not mode:
            resp.ok = False
            resp.message = "mode is empty"
            return resp

        try:
            ok = self._rebuild_recognizer(mode, custom_phrases=req.phrases)
            resp.ok = bool(ok)
            resp.message = f"grammar_mode set to {self._grammar_mode}"
        except Exception as e:
            resp.ok = False
            resp.message = f"failed: {e}"
        return resp

    # -------------------- Publish helpers --------------------
    def _publish_final(self, text: str):
        msg = String()
        msg.data = text
        self.pub_text.publish(msg)
        self.get_logger().info(f"[vosk] FINAL: {text}")

    def _publish_partial(self, partial: str):
        if not self.publish_partial:
            return
        if partial and partial != self._last_partial:
            self._last_partial = partial
            msg = String()
            msg.data = partial
            self.pub_partial.publish(msg)

    # -------------------- Audio callback --------------------
    def _audio_cb(self, msg: AudioData):
        chunk = bytes(msg.data)

        with self._rec_lock:
            rec = self.rec  # local ref under lock

            if rec.AcceptWaveform(chunk):
                try:
                    result = json.loads(rec.Result())
                except json.JSONDecodeError:
                    return

                text = (result.get("text") or "").strip()
                if text:
                    self._publish_final(text)

                self._last_partial = ""
            else:
                try:
                    pres = json.loads(rec.PartialResult())
                except json.JSONDecodeError:
                    return
                partial = (pres.get("partial") or "").strip()
                self._publish_partial(partial)


def main():
    rclpy.init()
    node = VoskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
