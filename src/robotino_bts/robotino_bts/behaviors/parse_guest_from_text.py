# robotino_bts/behaviors/parse_guest_from_text.py

import re
import py_trees
from py_trees.common import Access


class ParseGuestFromText(py_trees.behaviour.Behaviour):
    """
    Parse 'last_text' from the blackboard and fill:
      - current_guest_name
      - current_guest_drink

    This is intentionally simple + deterministic (no LLM).
    It matches against known_names/known_drinks lists (passed in).
    """

    def __init__(
        self,
        name: str,
        known_names: list[str],
        known_drinks: list[str],
        text_key: str = "last_text",
        out_name_key: str = "current_guest_name",
        out_drink_key: str = "current_guest_drink",
    ):
        super().__init__(name)

        self.known_names = [n.lower() for n in known_names]
        self.known_drinks = [d.lower() for d in known_drinks]

        self.text_key = text_key
        self.out_name_key = out_name_key
        self.out_drink_key = out_drink_key

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(key=self.text_key, access=Access.READ)
        self.bb.register_key(key=self.out_name_key, access=Access.WRITE)
        self.bb.register_key(key=self.out_drink_key, access=Access.WRITE)

    @staticmethod
    def _normalize(t: str) -> str:
        t = (t or "").lower().strip()
        t = re.sub(r"[^a-z0-9\s]", " ", t)
        t = re.sub(r"\s+", " ", t).strip()

        # cheap synonyms
        t = t.replace("coca cola", "coke")
        t = t.replace("cola", "coke")
        t = t.replace("iced tea", "tea")
        return t

    def _extract_name(self, text: str) -> str:
        # patterns
        m = re.search(r"\bmy name is\s+([a-z0-9]+)\b", text)
        if m and m.group(1) in self.known_names:
            return m.group(1)

        m = re.search(r"\bi am\s+([a-z0-9]+)\b", text)
        if m and m.group(1) in self.known_names:
            return m.group(1)

        m = re.search(r"\bthis is\s+([a-z0-9]+)\b", text)
        if m and m.group(1) in self.known_names:
            return m.group(1)

        # fallback: any token
        for tok in text.split():
            if tok in self.known_names:
                return tok

        return ""

    def _extract_drink(self, text: str) -> str:
        found = ""
        for tok in text.split():
            if tok in self.known_drinks:
                found = tok
        return found  # last match wins

    def update(self):
        if not self.bb.exists(self.text_key):
            self.feedback_message = f"BB missing '{self.text_key}'"
            return py_trees.common.Status.FAILURE

        raw = self.bb.get(self.text_key) or ""
        text = self._normalize(raw)

        name = self._extract_name(text)
        drink = self._extract_drink(text)

        # Only overwrite if we found something (so partial info can accumulate)
        if name:
            self.bb.set(self.out_name_key, name)
        if drink:
            self.bb.set(self.out_drink_key, drink)

        self.feedback_message = f"text='{text}' -> name='{name}' drink='{drink}'"

        # Success if both exist and are non-empty
        out_name = self.bb.get(self.out_name_key) if self.bb.exists(self.out_name_key) else ""
        out_drink = self.bb.get(self.out_drink_key) if self.bb.exists(self.out_drink_key) else ""

        if (out_name or "") != "" and (out_drink or "") != "":
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
