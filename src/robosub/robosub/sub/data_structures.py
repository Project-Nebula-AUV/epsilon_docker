#!/usr/bin/env python3
"""
Core data structures for the RoboSub control stack.

SensorSuite is the single object passed from the hardware/simulator layer
into the sub control stack each tick. It contains everything the sub needs
to make decisions — camera image, depth, heading, IMU, and velocities.

ThrusterCommands is the single object returned by the sub each tick.
The hardware/simulator layer is responsible for translating these into
actual motor commands or physics forces.

No pygame, no ROS — this module has zero external dependencies beyond numpy.
The simulator converts its pygame Surface to numpy before packaging a
SensorSuite. The hardware node receives numpy directly from cv_bridge.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple, Callable
import numpy as np

from robosub.sub.vision import find_blobs_hsv
from robosub.sub.config import MARKER_HSV_RANGE, RED_HSV_RANGES


@dataclass
class MPU6050Readings:
    """Raw readings from the IMU."""
    accel_x: float = 0.0   # Sway acceleration (m/s²)
    accel_y: float = 0.0   # Surge acceleration (m/s²)
    accel_z: float = 0.0   # Heave acceleration (m/s²)
    gyro_x:  float = 0.0   # Roll rate (rad/s)
    gyro_y:  float = 0.0   # Pitch rate (rad/s) — measured but not controlled
    gyro_z:  float = 0.0   # Yaw rate (rad/s)


@dataclass
class SensorSuite:
    """
    All sensor data for one control tick.

    camera_image: BGR numpy array (H x W x 3, uint8).
                  On sim: converted from pygame Surface by the simulator node.
                  On hardware: delivered directly from cv_bridge.

    velocity_x/y/z: Body-frame velocities in m/s.
                    On sim: ground truth from physics engine.
                    On hardware: from DVL, optical flow, or IMU integration
                    depending on available sensors.
    """
    camera_image:   np.ndarray
    depth:          float           # Meters below surface
    heading:        float           # Degrees, 0-360
    roll:           float           # Degrees, port-down positive
    imu:            MPU6050Readings
    velocity_x:     float = 0.0    # World-frame X velocity (m/s)
    velocity_y:     float = 0.0    # World-frame Y velocity (m/s)
    velocity_z:     float = 0.0    # Vertical velocity, down positive (m/s)


class Vision:
    """
    Handles vision processing for one control tick.

    Stores an image provider callable, runs blob detection when update()
    is called, and exposes interpretation methods to the task code.

    The image provider should return a BGR numpy array or None.
    Task code never touches raw images — it only calls the is_*/get_*
    methods below.
    """

    def __init__(self,
                 image_provider: Callable[[], Optional[np.ndarray]],
                 min_pole_pixels: int = 20,
                 min_gate_pixels: int = 50):

        if not callable(image_provider):
            raise TypeError("image_provider must be a callable")

        self.image_provider  = image_provider
        self.min_pole_pixels = min_pole_pixels
        self.min_gate_pixels = min_gate_pixels

        # marker_blobs: the course marker channel (WHITE on the 2026 course
        # — slalom whites and the orbit marker; see config.MARKER_HSV_RANGE).
        self.marker_blobs: List[Dict] = []
        self.red_blobs:    List[Dict] = []

        self._clear_cache()

    @property
    def green_blobs(self) -> List[Dict]:
        """Legacy alias for marker_blobs."""
        return self.marker_blobs

    def _clear_cache(self):
        self._best_green_pole:       Optional[Dict]              = None
        self._found_best_green_pole: bool                        = False
        self._best_gate_pair:        Optional[Tuple[Dict, Dict]] = None
        self._found_best_gate_pair:  bool                        = False

    def update(self):
        """
        Fetch the current camera image, run blob detection, clear caches.
        Call once per control tick before any task code runs.
        """
        img = self.image_provider()
        if img is None:
            self.marker_blobs = []
            self.red_blobs    = []
        else:
            # Min-area thresholds are calibrated in SIM-REFERENCE pixels
            # (320x240). The hardware camera publishes 640x320 — 2.67x the
            # pixel count — so scale by the actual image area or every
            # threshold is silently ~2.7x stricter in the water than in the
            # sim (W6 2026-07-07; sysid RESUME "pixel-count 2x scale" item).
            h, w = img.shape[:2]
            scale = (h * w) / (240.0 * 320.0)
            self.marker_blobs = find_blobs_hsv(
                img, MARKER_HSV_RANGE,
                max(1, int(round(self.min_pole_pixels * scale))))
            self.red_blobs    = find_blobs_hsv(
                img, RED_HSV_RANGES,
                max(1, int(round(self.min_gate_pixels * scale))))
        self._clear_cache()

    # --- Green pole (marker) ---

    def get_best_pole(self) -> Optional[Dict]:
        if not self._found_best_green_pole:
            self._found_best_green_pole = True
            candidates = [b for b in self.marker_blobs
                          if b['height'] > b['width'] * 1.2]
            self._best_green_pole = (max(candidates, key=lambda b: b['area'])
                                     if candidates else None)
        return self._best_green_pole

    def is_pole_visible(self) -> bool:
        return self.get_best_pole() is not None

    def get_pole_center_x(self) -> Optional[float]:
        pole = self.get_best_pole()
        return pole['center_x'] if pole else None

    def get_pole_apparent_height(self) -> float:
        pole = self.get_best_pole()
        return pole['height'] if pole else 0.0

    def get_pole_apparent_width(self) -> float:
        pole = self.get_best_pole()
        return pole['width'] if pole else 0.0

    # --- Red poles (gate) ---

    def _is_slalom_red(self, r: Dict) -> bool:
        """A red pole with a white pole beside it at similar bottom height AND
        similar size is a slalom gatelet red, NOT a gate pole. Without this
        exclusion, two slalom reds from different gatelets pair up as a
        phantom gate and DriveUntilTargetLost keeps surging through the
        slalom field. The size-ratio gate matters: a real gatelet's white is
        at the same distance as its red (height ratio ~1), while a DISTANT
        slalom white can coincidentally line up with a NEAR gate red's max_y
        (seen at 1.5 m mission depth) — the 4x height difference is what
        tells them apart."""
        for w in self.marker_blobs:
            if w['height'] <= w['width'] * 1.2:
                continue
            # W6b (2026-07-08): a TRUE gatelet is the same pole type at the
            # same distance — ratio ~1.0, lateral separation ~1.7x height
            # (1.524 m gap / 0.9 m pole), bottoms tightly aligned. The old
            # loose bands (0.5-2.0 ratio, 0-2.5h separation, 0.6h bottoms)
            # let EVERY red find SOME white on the return leg (4+ slalom
            # whites in frame) — the real gate posts were all excluded and
            # the return gate could never be acquired.
            ratio = w['height'] / max(r['height'], 1)
            if not (0.7 <= ratio <= 1.45):
                continue
            mean_h = 0.5 * (r['height'] + w['height'])
            sep = abs(r['center_x'] - w['center_x'])
            if not (0.8 * mean_h <= sep <= 2.4 * mean_h):
                continue
            if abs(r['max_y'] - w['max_y']) < max(r['height'], w['height']) * 0.35:
                return True
        return False

    def get_gate_post_blobs(self) -> List[Dict]:
        """Red blobs that could plausibly be GATE POSTS: vertical aspect
        (excludes the 2026 maker boxes) and not a slalom-gatelet red. This is
        the ONLY red set gate behaviors should steer by — blind fallbacks
        that chased raw red_blobs went after the red maker box (W6)."""
        return [b for b in self.red_blobs
                if b['height'] > b['width'] * 1.5
                and not self._is_slalom_red(b)]

    def get_gate_pair(self) -> Optional[Tuple[Dict, Dict]]:
        if not self._found_best_gate_pair:
            self._found_best_gate_pair = True
            candidates = self.get_gate_post_blobs()
            if len(candidates) < 2:
                self._best_gate_pair = None
            else:
                candidates.sort(key=lambda p: p['center_x'])
                best, min_diff = None, float('inf')
                # ALL pairs, not adjacent-only: the 2026 gate hangs a RED
                # divider between the posts, so the x-sorted sequence is
                # (L, divider, R) and the true pair is non-adjacent. The
                # divider itself never survives the height-ratio filter
                # (0.61 m plate vs 1.5 m posts).
                for i in range(len(candidates) - 1):
                    for j in range(i + 1, len(candidates)):
                        p1, p2 = candidates[i], candidates[j]
                        h_min = min(p1['height'], p2['height'])
                        h_max = max(p1['height'], p2['height'])
                        h_mean = 0.5 * (p1['height'] + p2['height'])
                        y_overlap = (min(p1['max_y'], p2['max_y'])
                                     - max(p1['min_y'], p2['min_y']))
                        sep = p2['center_x'] - p1['center_x']
                        # Real gate posts sit at the same distance: similar
                        # heights (ratio, not an absolute pixel diff) and
                        # substantial vertical overlap. AND the gate has a
                        # fixed shape: width/post-height = 2.0 (3.0/1.5 comp,
                        # 2.0/1.0 pool practice gate), compressed by oblique
                        # viewing — so separation must be ~0.8-3.0x the mean
                        # height. Without this, two IN-LINE slalom reds at
                        # different distances (tiny separation, similar
                        # apparent heights) pair as a phantom gate and
                        # DriveUntilTargetLost chases them into the slalom
                        # field (w6_full_hw1: 15 m overshoot, mission fail).
                        if (y_overlap > 0.5 * h_min and h_min > 0.55 * h_max
                                and 0.8 * h_mean <= sep <= 3.0 * h_mean):
                            h_diff = h_max - h_min
                            if h_diff < min_diff:
                                min_diff = h_diff
                                best = (p1, p2)
                self._best_gate_pair = best
        return self._best_gate_pair

    def is_gate_visible(self) -> bool:
        return self.get_gate_pair() is not None

    def get_gate_center_x(self) -> Optional[float]:
        pair = self.get_gate_pair()
        return ((pair[0]['center_x'] + pair[1]['center_x']) / 2
                if pair else None)

    # --- Slalom gatelets (one red pole + one white pole at similar height) ---
    # White poles come from the marker channel (MARKER_HSV_RANGE = white).

    def get_slalom_gatelet(self, pass_side: Optional[str] = None
                           ) -> Optional[Tuple[Dict, Dict]]:
        """
        Closest (lowest-in-image) red+white pole pair whose bottoms are at a
        similar image height. pass_side 'left'/'right' additionally requires
        the white pole on that side of the red pole. Returns (red, white).
        """
        reds = [b for b in self.red_blobs if b['height'] > b['width'] * 1.2]
        whites = [b for b in self.marker_blobs if b['height'] > b['width'] * 1.2]
        best, best_y = None, -1.0
        for r in reds:
            for w in whites:
                # A genuine gatelet's red and white stand at the SAME
                # distance: similar apparent heights, aligned bottoms, and a
                # pixel separation set by the course geometry (1.524 m gap /
                # ~1.3 m pole ≈ 1.2x the apparent height). Without the size
                # and separation gates, a NEAR red pairs with a FAR white
                # from the next triplet sitting pixel-adjacent to it — the
                # "gap middle" then collapses onto the red pole (confirmed:
                # the vehicle slalomed straight down the red pole line).
                ratio = w['height'] / max(r['height'], 1)
                if not (0.55 <= ratio <= 1.8):
                    continue
                if abs(r['max_y'] - w['max_y']) > max(r['height'], w['height']) * 0.5:
                    continue
                sep = abs(w['center_x'] - r['center_x'])
                mean_h = (r['height'] + w['height']) / 2.0
                if not (0.6 * mean_h <= sep <= 2.2 * mean_h):
                    continue
                if pass_side == 'left' and w['center_x'] >= r['center_x']:
                    continue
                if pass_side == 'right' and w['center_x'] <= r['center_x']:
                    continue
                pair_y = max(r['max_y'], w['max_y'])
                # prefer the closest gatelet; tie-break on white nearest red
                if pair_y > best_y or (pair_y == best_y and best is not None
                                       and sep < abs(best[1]['center_x']
                                                     - best[0]['center_x'])):
                    best, best_y = (r, w), pair_y
        return best

    def is_slalom_visible(self, pass_side: Optional[str] = None) -> bool:
        return self.get_slalom_gatelet(pass_side) is not None


@dataclass
class ThrusterCommands:
    """
    Normalized thruster commands, range -1.0 to 1.0.

    Horizontal thrusters are vectored at 45 degrees at the four corners.
    Vertical thrusters are fore and aft.

    The simulator translates these into physics forces.
    The hardware node translates these into ESC/PWM signals.
    """
    hfl: float = 0.0   # Horizontal front-left
    hfr: float = 0.0   # Horizontal front-right
    hal: float = 0.0   # Horizontal aft-left
    har: float = 0.0   # Horizontal aft-right
    vp:  float = 0.0   # Vertical port (left)
    vs:  float = 0.0   # Vertical starboard (right)