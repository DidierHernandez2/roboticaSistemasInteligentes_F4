#!/usr/bin/env python3
import shutil
import subprocess
from typing import Optional, List

import rclpy
from rclpy.node import Node

from robotino_interfaces.srv import Talk 


class EspeakTTSNode(Node):
    """
    /tts/talk service (robotino_interfaces/Talk)
      Request:
        - text: string
        - wait: bool   (True = sync/block until finished, False = async)
      Response:
        - success: bool
        - message: string

    Params:
      - engine: string (""=auto, else "espeak-ng" or "espeak")
      - voice: string  (""=default, e.g. "en", "en-us", "es", etc.)
      - speed: int     (words per minute, typical 80-300)
      - pitch: int     (0-99)
      - volume: int    (0-200, depends on build)
      - interrupt: bool (if True, kill current speech when a new request arrives)
    """

    def __init__(self):
        super().__init__("espeak_tts_node")

        self.declare_parameter("engine", "")
        self.declare_parameter("voice", "")
        self.declare_parameter("speed", 140)
        self.declare_parameter("pitch", 120)
        self.declare_parameter("volume", 100)
        self.declare_parameter("interrupt", True)

        self._proc: Optional[subprocess.Popen] = None

        self._srv = self.create_service(Talk, "/tts/talk", self.cb_talk)
        self.get_logger().info("[TTS] /tts/talk service ready (robotino_interfaces/Talk)")

    def _pick_engine(self) -> Optional[str]:
        engine_param = self.get_parameter("engine").value.strip()
        if engine_param:
            return engine_param if shutil.which(engine_param) else None

        # auto preference
        if shutil.which("espeak-ng"):
            return "espeak-ng"
        if shutil.which("espeak"):
            return "espeak"
        return None

    def _build_cmd(self, engine: str, text: str) -> List[str]:
        voice = self.get_parameter("voice").value.strip()
        speed = int(self.get_parameter("speed").value)
        pitch = int(self.get_parameter("pitch").value)
        volume = int(self.get_parameter("volume").value)

        cmd = [engine]

        # Common flags in espeak/espeak-ng:
        # -v voice, -s speed, -p pitch, -a amplitude/volume
        if voice:
            cmd += ["-v", voice]
        if speed > 0:
            cmd += ["-s", str(speed)]
        if 0 <= pitch <= 99:
            cmd += ["-p", str(pitch)]
        if volume >= 0:
            cmd += ["-a", str(volume)]

        cmd.append(text)
        return cmd

    def _stop_current(self):
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass
        self._proc = None

    def cb_talk(self, req: Talk.Request, res: Talk.Response) -> Talk.Response:
        text = (req.text or "").strip()
        if not text:
            res.success = False
            res.message = "Empty text"
            return res

        engine = self._pick_engine()
        if not engine:
            res.success = False
            res.message = "No TTS engine found (install espeak-ng or espeak)"
            return res

        interrupt = bool(self.get_parameter("interrupt").value)
        if interrupt:
            self._stop_current()

        cmd = self._build_cmd(engine, text)
        self.get_logger().info(f"[TTS] {('SYNC' if req.wait else 'ASYNC')} cmd: {' '.join(cmd)}")

        try:
            if req.wait:
                # sync/blocking
                completed = subprocess.run(cmd, check=False)
                ok = (completed.returncode == 0)
                res.success = ok
                res.message = "Spoken (sync)" if ok else f"espeak returned {completed.returncode}"
                return res

            # async/non-blocking
            self._proc = subprocess.Popen(cmd)
            res.success = True
            res.message = "Speaking (async)"
            return res

        except Exception as e:
            res.success = False
            res.message = f"Exception: {e}"
            return res


def main():
    rclpy.init()
    node = EspeakTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_current()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
