#!/usr/bin/env python3
"""
camera_controls_tui.py — Live camera controls TUI
==================================================

A curses terminal UI that lets you adjust every camera parameter in real
time without restarting the node.

Dependencies
------------
  • rclpy  (ROS2 Python client)
  • PyYAML (pip install pyyaml)
  • stdlib: curses, json, threading

Usage
-----
  python camera_controls_tui.py [--params-file camera_params.yaml]

  # Or from a ROS2 workspace (no ros2 run needed — it's a standalone tool):
  python camera_controls_tui.py -p /path/to/camera_params.yaml

Keys
----
  ↑ / ↓          Select control
  ← / →          Decrease / increase value
  PgUp / PgDn    Large step (×10)
  Space           Toggle boolean controls
  s               Save current values to YAML file
  r               Reload values from YAML file (resets unsaved changes)
  q / Ctrl+C      Quit

What happens on change
----------------------
  1. The new value is published immediately to /camera/controls as JSON.
  2. The node receives the message and pushes the change to the V4L2 driver.
  3. When you press `s` the full state is written back to the YAML file.
"""

import argparse
import curses
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

# ── ROS2 import (optional: TUI still works without ROS for YAML editing) ──────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False


# ── Control definitions ───────────────────────────────────────────────────────

class Control:
    """Descriptor for one adjustable parameter."""
    __slots__ = ("key", "label", "kind", "min", "max", "step", "unit")

    def __init__(self, key: str, label: str, kind: str = "int",
                 min_val: int = -1, max_val: int = 255,
                 step: int = 1, unit: str = ""):
        self.key   = key
        self.label = label
        self.kind  = kind    # "int" | "bool"
        self.min   = min_val
        self.max   = max_val
        self.step  = step
        self.unit  = unit


CONTROLS: list[Control] = [
    # ── Exposure ──────────────────────────────────────────────────────────
    Control("auto_exposure", "Auto Exposure",   kind="bool"),
    Control("exposure",      "Exposure",        kind="int",
            min_val=-1, max_val=10000, step=10, unit="×100µs"),

    # ── White balance ─────────────────────────────────────────────────────
    Control("auto_wb",       "Auto WB",         kind="bool"),
    Control("wb_temperature","WB Temperature",  kind="int",
            min_val=-1, max_val=10000, step=100, unit="K"),

    # ── V4L2 scalars ──────────────────────────────────────────────────────
    Control("brightness",    "Brightness",      kind="int",  min_val=-1, max_val=255),
    Control("contrast",      "Contrast",        kind="int",  min_val=-1, max_val=255),
    Control("saturation",    "Saturation",      kind="int",  min_val=-1, max_val=255),
    Control("gain",          "Gain",            kind="int",  min_val=-1, max_val=255),
    Control("sharpness",     "Sharpness",       kind="int",  min_val=-1, max_val=255),

    # ── Software ──────────────────────────────────────────────────────────
    Control("gray_world",    "Gray-World SW WB",kind="bool"),
]

PARAM_KEY = ("camera_input", "ros__parameters")  # path inside the YAML


# ── YAML helpers ──────────────────────────────────────────────────────────────

def load_yaml(path: Path) -> dict[str, Any]:
    """Load the params YAML and return the ros__parameters dict."""
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    return doc.get(PARAM_KEY[0], {}).get(PARAM_KEY[1], {})


def save_yaml(path: Path, params: dict[str, Any]) -> None:
    """Merge *params* back into the YAML file, preserving comments where possible."""
    # Read the raw file to keep structure; overlay our values.
    if path.exists():
        with open(path) as fh:
            doc = yaml.safe_load(fh) or {}
    else:
        doc = {}

    doc.setdefault(PARAM_KEY[0], {}).setdefault(PARAM_KEY[1], {}).update(params)

    with open(path, "w") as fh:
        yaml.dump(doc, fh, default_flow_style=False, sort_keys=False)


# ── ROS2 publisher (runs in a background thread) ──────────────────────────────

class ControlPublisher:
    """Thin wrapper: spin a ROS2 node in a daemon thread, expose publish()."""

    def __init__(self):
        self._node: Node | None = None
        self._pub  = None
        self._lock = threading.Lock()
        self._ok   = False

        if not _HAS_ROS:
            return

        t = threading.Thread(target=self._spin, daemon=True)
        t.start()
        # Give rclpy a moment to initialise before the TUI starts
        time.sleep(0.4)

    def _spin(self):
        try:
            rclpy.init()
            self._node = rclpy.create_node("camera_controls_tui")
            self._pub  = self._node.create_publisher(String, "/camera/controls", 10)
            self._ok   = True
            rclpy.spin(self._node)
        except Exception:
            pass

    def publish(self, updates: dict) -> bool:
        if not self._ok or self._pub is None:
            return False
        with self._lock:
            msg      = String()
            msg.data = json.dumps(updates)
            self._pub.publish(msg)
        return True

    def shutdown(self):
        if self._node:
            self._node.destroy_node()
        if _HAS_ROS and rclpy.ok():
            rclpy.shutdown()

    @property
    def connected(self) -> bool:
        return self._ok


# ── Curses TUI ────────────────────────────────────────────────────────────────

class TUI:
    # Colour pair indices
    CP_NORMAL    = 1   # unselected row text
    CP_SEL_ROW   = 2   # selected row highlight
    CP_HEADER    = 3   # title bar
    CP_STATUS_OK = 4   # status bar — ok
    CP_WARN      = 5   # status bar — warning / dirty flag
    CP_DIM       = 6   # separators, hints
    CP_VAL_ON    = 7   # bool ON / numeric value (bright)
    CP_VAL_OFF   = 8   # bool OFF / disabled (-1)
    CP_BAR_FILL  = 9   # filled portion of progress bar
    CP_BAR_EMPTY = 10  # empty portion of progress bar
    CP_SECTION   = 11  # section header labels
    CP_KEY       = 12  # key names in hint bar
    CP_SEL_VAL   = 13  # value column on selected row

    # Column positions (computed relative to terminal width in _draw)
    COL_INDICATOR = 0   # ▶ / space
    COL_LABEL     = 2   # parameter name
    COL_VALUE     = 22  # numeric / bool value
    COL_BAR       = 32  # progress bar
    COL_UNIT      = 57  # unit string
    MIN_WIDTH     = 72  # below this we suppress bar + unit

    # Section dividers inserted before these control indices
    SECTIONS: dict[int, str] = {
        0: "EXPOSURE",
        2: "WHITE BALANCE",
        4: "V4L2 SCALARS",
        9: "SOFTWARE",
    }

    def __init__(self, stdscr, params_path: Path, publisher: ControlPublisher):
        self.scr    = stdscr
        self.path   = params_path
        self.pub    = publisher
        self.cursor = 0
        self.status      = "Ready"
        self.status_warn = False
        self.dirty  = False

        curses.start_color()
        curses.use_default_colors()
        curses.curs_set(0)

        # Background -1 = terminal default (transparent)
        curses.init_pair(self.CP_NORMAL,    curses.COLOR_WHITE,   -1)
        curses.init_pair(self.CP_SEL_ROW,   curses.COLOR_BLACK,   curses.COLOR_CYAN)
        curses.init_pair(self.CP_HEADER,    curses.COLOR_BLACK,   curses.COLOR_CYAN)
        curses.init_pair(self.CP_STATUS_OK, curses.COLOR_BLACK,   curses.COLOR_GREEN)
        curses.init_pair(self.CP_WARN,      curses.COLOR_BLACK,   curses.COLOR_YELLOW)
        curses.init_pair(self.CP_DIM,       curses.COLOR_WHITE,   -1)
        curses.init_pair(self.CP_VAL_ON,    curses.COLOR_GREEN,   -1)
        curses.init_pair(self.CP_VAL_OFF,   curses.COLOR_WHITE,   -1)
        curses.init_pair(self.CP_BAR_FILL,  curses.COLOR_CYAN,    -1)
        curses.init_pair(self.CP_BAR_EMPTY, curses.COLOR_WHITE,   -1)
        curses.init_pair(self.CP_SECTION,   curses.COLOR_YELLOW,  -1)
        curses.init_pair(self.CP_KEY,       curses.COLOR_CYAN,    -1)
        curses.init_pair(self.CP_SEL_VAL,   curses.COLOR_CYAN,    curses.COLOR_BLACK)

        self.values = self._load()

    # ── Data ──────────────────────────────────────────────────────────────

    def _load(self) -> dict[str, Any]:
        defaults: dict[str, Any] = {
            "auto_exposure":  True,
            "exposure":       -1,
            "auto_wb":        True,
            "wb_temperature": -1,
            "brightness":     -1,
            "contrast":       -1,
            "saturation":     -1,
            "gain":           -1,
            "sharpness":      -1,
            "gray_world":     False,
        }
        if self.path.exists():
            try:
                on_disk = load_yaml(self.path)
                for k in defaults:
                    if k in on_disk:
                        defaults[k] = on_disk[k]
                self._set_status(f"Loaded  {self.path.name}", warn=False)
            except Exception as exc:
                self._set_status(f"Load error: {exc}", warn=True)
        return defaults

    def _save(self) -> None:
        try:
            save_yaml(self.path, self.values)
            self.dirty = False
            self._set_status(f"Saved   {self.path}", warn=False)
        except Exception as exc:
            self._set_status(f"Save error: {exc}", warn=True)

    def _reload(self) -> None:
        self.values = self._load()
        self.dirty  = False

    def _set_status(self, msg: str, warn: bool = False) -> None:
        self.status      = msg
        self.status_warn = warn

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish(self, key: str, value: Any) -> None:
        sent = self.pub.publish({key: value})
        if not sent:
            self._set_status("ROS2 offline — change saved locally only", warn=True)

    # ── Value manipulation ────────────────────────────────────────────────

    def _get_ctrl(self) -> Control:
        return CONTROLS[self.cursor]

    def _adjust(self, delta: int) -> None:
        ctrl = self._get_ctrl()
        if ctrl.kind == "bool":
            new_val = not self.values.get(ctrl.key, False)
        else:
            current = self.values.get(ctrl.key, -1)
            if current == -1 and delta > 0:
                new_val = ctrl.step
            elif current != -1 and current - ctrl.step < max(ctrl.min, 0) and delta < 0:
                new_val = -1
            else:
                new_val = max(ctrl.min, min(ctrl.max, current + delta * ctrl.step))

        self.values[ctrl.key] = new_val
        self.dirty = True
        self._publish(ctrl.key, new_val)
        if ctrl.kind == "bool":
            self._set_status(f"{ctrl.label}  →  {'ON' if new_val else 'OFF'}")
        else:
            disp = "disabled (-1)" if new_val == -1 else f"{new_val}{(' ' + ctrl.unit) if ctrl.unit else ''}"
            self._set_status(f"{ctrl.label}  →  {disp}")

    # ── Rendering helpers ─────────────────────────────────────────────────

    def _safe_addstr(self, row: int, col: int, text: str, attr: int = 0) -> None:
        """addstr that silently swallows out-of-bounds errors."""
        h, w = self.scr.getmaxyx()
        if row < 0 or row >= h or col < 0 or col >= w:
            return
        text = text[:max(0, w - col - 1)]
        if not text:
            return
        try:
            self.scr.addstr(row, col, text, attr)
        except curses.error:
            pass

    def _hline(self, row: int, char: str = "─") -> None:
        _, w = self.scr.getmaxyx()
        self._safe_addstr(row, 0, char * (w - 1), curses.color_pair(self.CP_DIM))

    def _bar_segments(self, value: int, min_val: int, max_val: int,
                      width: int = 22) -> tuple[int, int]:
        """Return (filled_chars, empty_chars) for a progress bar."""
        if value < 0:
            return 0, width
        effective_min = max(min_val, 0)
        ratio  = (value - effective_min) / max(max_val - effective_min, 1)
        filled = max(0, min(width, int(ratio * width)))
        return filled, width - filled

    def _draw_bar(self, row: int, col: int, value: int,
                  ctrl: Control, selected: bool, width: int = 22) -> None:
        """Draw a colour-segmented progress bar in-place."""
        filled, empty = self._bar_segments(value, ctrl.min, ctrl.max, width)
        bar_attr_fill  = (curses.color_pair(self.CP_SEL_ROW)   if selected
                          else curses.color_pair(self.CP_BAR_FILL) | curses.A_BOLD)
        bar_attr_empty = (curses.color_pair(self.CP_SEL_ROW)   if selected
                          else curses.color_pair(self.CP_BAR_EMPTY))
        dot_attr       = (curses.color_pair(self.CP_SEL_ROW)   if selected
                          else curses.color_pair(self.CP_DIM))

        self._safe_addstr(row, col, "▕", curses.color_pair(self.CP_DIM))
        if value < 0:
            self._safe_addstr(row, col + 1, "·" * width, dot_attr)
        else:
            self._safe_addstr(row, col + 1, "█" * filled, bar_attr_fill)
            self._safe_addstr(row, col + 1 + filled, "░" * empty, bar_attr_empty)
        self._safe_addstr(row, col + 1 + width, "▏", curses.color_pair(self.CP_DIM))

    def _draw(self) -> None:
        self.scr.erase()
        h, w = self.scr.getmaxyx()
        wide = w >= self.MIN_WIDTH
        bar_width = max(10, min(22, w - self.COL_BAR - 12))

        # ── Row 0: Title bar (full-width highlight) ───────────────────────
        title = f" Arducam OV9782  Camera Controls "
        self._safe_addstr(0, 0, title.center(w - 1),
                          curses.color_pair(self.CP_HEADER) | curses.A_BOLD)

        # ── Row 1: ROS2 status  |  file name ─────────────────────────────
        ros_str  = ("● ROS2 connected" if self.pub.connected
                    else "○ ROS2 offline")
        ros_attr = (curses.color_pair(self.CP_VAL_ON)  | curses.A_BOLD if self.pub.connected
                    else curses.color_pair(self.CP_WARN) | curses.A_BOLD)
        self._safe_addstr(1, 2, ros_str, ros_attr)

        file_str  = f"{self.path.name}{'*' if self.dirty else ''}"
        file_attr = (curses.color_pair(self.CP_WARN) | curses.A_BOLD if self.dirty
                     else curses.color_pair(self.CP_DIM))
        self._safe_addstr(1, max(0, w - len(file_str) - 2), file_str, file_attr)

        # ── Row 2: Column headers ─────────────────────────────────────────
        self._hline(2)
        self._safe_addstr(2, self.COL_LABEL, "PARAMETER",
                          curses.color_pair(self.CP_DIM) | curses.A_BOLD)
        self._safe_addstr(2, self.COL_VALUE, "VALUE",
                          curses.color_pair(self.CP_DIM) | curses.A_BOLD)
        if wide:
            self._safe_addstr(2, self.COL_BAR, "RANGE",
                              curses.color_pair(self.CP_DIM) | curses.A_BOLD)

        # ── Rows 3+: Controls (with section dividers) ─────────────────────
        row = 3
        for i, ctrl in enumerate(CONTROLS):
            if row >= h - 5:
                break

            # Section divider
            if i in self.SECTIONS:
                if row >= h - 5:
                    break
                label = f"  {self.SECTIONS[i]}  "
                self._safe_addstr(row, 0, "─" * 2 + label,
                                  curses.color_pair(self.CP_SECTION) | curses.A_BOLD)
                self._safe_addstr(row, 2 + len(label),
                                  "─" * max(0, w - 3 - len(label)),
                                  curses.color_pair(self.CP_DIM))
                row += 1
                if row >= h - 5:
                    break

            selected = (i == self.cursor)
            value    = self.values.get(ctrl.key)

            # Full-row background for selected
            if selected:
                self._safe_addstr(row, 0, " " * (w - 1),
                                  curses.color_pair(self.CP_SEL_ROW))

            # ▶ indicator
            indicator = "▶" if selected else " "
            ind_attr  = (curses.color_pair(self.CP_SEL_ROW) | curses.A_BOLD if selected
                         else curses.color_pair(self.CP_DIM))
            self._safe_addstr(row, self.COL_INDICATOR, indicator, ind_attr)

            # Label
            label_attr = (curses.color_pair(self.CP_SEL_ROW) | curses.A_BOLD if selected
                          else curses.color_pair(self.CP_NORMAL))
            self._safe_addstr(row, self.COL_LABEL, ctrl.label[:18].ljust(18), label_attr)

            # Value column — coloured independently even on selected row
            if ctrl.kind == "bool":
                if value:
                    val_str  = "ON "
                    val_attr = (curses.color_pair(self.CP_SEL_ROW) | curses.A_BOLD if selected
                                else curses.color_pair(self.CP_VAL_ON) | curses.A_BOLD)
                else:
                    val_str  = "OFF"
                    val_attr = (curses.color_pair(self.CP_SEL_ROW) if selected
                                else curses.color_pair(self.CP_VAL_OFF))
                self._safe_addstr(row, self.COL_VALUE, f"[{val_str}]", val_attr)
            else:
                if value is None or value < 0:
                    val_str  = "  --"
                    val_attr = (curses.color_pair(self.CP_SEL_ROW) if selected
                                else curses.color_pair(self.CP_DIM))
                else:
                    val_str  = f"{value:4d}"
                    val_attr = (curses.color_pair(self.CP_SEL_ROW) | curses.A_BOLD if selected
                                else curses.color_pair(self.CP_VAL_ON) | curses.A_BOLD)
                self._safe_addstr(row, self.COL_VALUE, val_str, val_attr)

                if wide and ctrl.kind == "int":
                    # Draw bar + unit
                    self._draw_bar(row, self.COL_BAR, value if value is not None else -1,
                                   ctrl, selected, bar_width)
                    if ctrl.unit:
                        unit_col = self.COL_BAR + bar_width + 3
                        unit_attr = (curses.color_pair(self.CP_SEL_ROW) if selected
                                     else curses.color_pair(self.CP_DIM))
                        self._safe_addstr(row, unit_col, ctrl.unit, unit_attr)

            row += 1

        # ── Detail panel for selected control ─────────────────────────────
        panel_top = h - 5
        self._hline(panel_top)

        ctrl  = self._get_ctrl()
        value = self.values.get(ctrl.key)

        # Line 1: full parameter key + current value + range
        if ctrl.kind == "bool":
            detail_val = "ON" if value else "OFF"
            range_str  = "toggle with  Space  or  ← →"
        else:
            detail_val = str(value) if value is not None and value >= 0 else "-1 (disabled)"
            range_str  = (f"range {ctrl.min}…{ctrl.max}   step {ctrl.step}"
                          + (f"   unit {ctrl.unit}" if ctrl.unit else ""))

        self._safe_addstr(panel_top + 1, 2,
                          f"{ctrl.key}", curses.color_pair(self.CP_KEY) | curses.A_BOLD)
        self._safe_addstr(panel_top + 1, 2 + len(ctrl.key) + 2,
                          f"=  {detail_val}",
                          curses.color_pair(self.CP_VAL_ON) | curses.A_BOLD)
        self._safe_addstr(panel_top + 1, max(0, w - len(range_str) - 2),
                          range_str, curses.color_pair(self.CP_DIM))

        # Line 2: wide bar spanning most of the terminal width
        if ctrl.kind == "int" and wide:
            big_bar_w = max(10, w - 8)
            self._draw_bar(panel_top + 2, 3,
                           value if value is not None else -1,
                           ctrl, False, big_bar_w)

        # ── Hint bar ──────────────────────────────────────────────────────
        self._hline(h - 2)
        hints = [
            ("↑↓",    "select"),
            ("←→",    "adjust ×1"),
            ("PgUp↑", "adjust ×10"),
            ("Spc",   "toggle"),
            ("s",     "save"),
            ("r",     "reload"),
            ("q",     "quit"),
        ]
        col = 1
        for key_str, desc in hints:
            if col >= w - 2:
                break
            self._safe_addstr(h - 1, col, key_str,
                              curses.color_pair(self.CP_KEY) | curses.A_BOLD)
            col += len(key_str)
            label_part = f" {desc}  "
            self._safe_addstr(h - 1, col, label_part,
                              curses.color_pair(self.CP_DIM))
            col += len(label_part)

        # ── Status bar (row h-2, right-aligned alongside hints) ───────────
        status_attr = (curses.color_pair(self.CP_WARN)      | curses.A_BOLD
                       if self.status_warn
                       else curses.color_pair(self.CP_STATUS_OK) | curses.A_BOLD)
        status_text = f" {self.status} "
        self._safe_addstr(h - 2, max(0, w - len(status_text) - 1),
                          status_text, status_attr)

        self.scr.refresh()

    # ── Main loop ─────────────────────────────────────────────────────────

    def run(self) -> None:
        self.scr.keypad(True)
        self.scr.timeout(150)

        while True:
            self._draw()
            key = self.scr.getch()

            if key in (ord("q"), ord("Q"), 27):
                break
            elif key in (curses.KEY_UP, ord("k")):
                self.cursor = max(0, self.cursor - 1)
            elif key in (curses.KEY_DOWN, ord("j")):
                self.cursor = min(len(CONTROLS) - 1, self.cursor + 1)
            elif key in (curses.KEY_LEFT, ord("h")):
                self._adjust(-1)
            elif key in (curses.KEY_RIGHT, ord("l")):
                self._adjust(+1)
            elif key == curses.KEY_PPAGE:
                self._adjust(+10)
            elif key == curses.KEY_NPAGE:
                self._adjust(-10)
            elif key == ord(" "):
                ctrl = self._get_ctrl()
                if ctrl.kind == "bool":
                    self._adjust(0)
            elif key in (ord("s"), ord("S")):
                self._save()
            elif key in (ord("r"), ord("R")):
                self._reload()


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Camera controls TUI")
    p.add_argument(
        "-p", "--params-file",
        default="camera_params.yaml",
        help="Path to the ROS2 params YAML file (default: camera_params.yaml)",
    )
    return p.parse_args()


def main():
    args        = parse_args()
    params_path = Path(args.params_file)

    if not params_path.exists():
        print(f"Warning: {params_path} not found — will be created on first save.")

    publisher = ControlPublisher()

    try:
        curses.wrapper(lambda stdscr: TUI(stdscr, params_path, publisher).run())
    except KeyboardInterrupt:
        pass
    finally:
        publisher.shutdown()

    print("Bye.")


if __name__ == "__main__":
    main()
