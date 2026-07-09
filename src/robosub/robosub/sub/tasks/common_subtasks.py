#!/usr/bin/env python3
"""Reusable mission subtasks.

Design rules (see control.py for the law definitions):
  * the compass owns heading, vision owns lateral position — no subtask
    yaw-steers onto a biased pixel target;
  * completion criteria only use signals the real vehicle can sense
    (pixel error / pixel rate, gyro, fused depth and vertical velocity,
    elapsed time) — never ground-truth lateral velocity;
  * every subtask is bounded (TIMEOUT + ON_TIMEOUT policy, see
    subtask_base.py).

Gate geometry: with the heading locked square to the gate, the aim point for
a committed side is the pixel midpoint between the gate center and the post
on that side. Putting that point at the frame center with pure sway places
the vehicle on the half-opening's axis. Image-right ('right' side) is the
vehicle's starboard half when square to the gate.
"""
import math
from typing import Dict, Any, Optional, Tuple

import numpy as np

from robosub.sub.tasks.subtask_base import Subtask, SubtaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig
from robosub.sub.utils import angle_diff


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _is_visible(vision: Vision, target_type: str) -> bool:
    if target_type == 'gate':
        return vision.is_gate_visible()
    if target_type == 'pole':
        return vision.is_pole_visible()
    return vision.is_gate_visible() or vision.is_pole_visible()


def _target_center_x(vision: Vision) -> Optional[float]:
    if vision.is_gate_visible():
        return vision.get_gate_center_x()
    if vision.is_pole_visible():
        return vision.get_pole_center_x()
    return None


def gate_half_center_px(pair: Tuple[Dict, Dict], side: str) -> float:
    """Pixel x of the center of the chosen half-opening ('left'/'right' in
    image space; pair is (left_post, right_post) sorted by center_x)."""
    c_l, c_r = pair[0]['center_x'], pair[1]['center_x']
    center = (c_l + c_r) / 2.0
    return (center + c_r) / 2.0 if side == 'right' else (center + c_l) / 2.0


def _err_norm(px: float, cam_w: int) -> float:
    return (px - cam_w / 2.0) / (cam_w / 2.0)


# ---------------------------------------------------------------------------
# reference & basic motion
# ---------------------------------------------------------------------------

class CaptureReference(Subtask):
    """Locks the course axis for this task into context['axis'].

    The mission-wide reference (gate normal) is captured once, on the first
    gate approach, from the vehicle's current heading — the vehicle starts
    square to the gate. A reversed task uses the reciprocal axis.
    """
    TIMEOUT = 5.0
    ON_TIMEOUT = 'complete'

    def __init__(self, reverse: bool = False):
        super().__init__()
        self.reverse = reverse

    def execute(self, sub, dt, sensors, vision, config, context):
        if sub.shared.reference_heading is None:
            sub.shared.reference_heading = sensors.heading
            print(f"INFO: course reference locked at "
                  f"{sub.shared.reference_heading:.1f} deg")
        axis = sub.shared.reference_heading
        if self.reverse:
            axis = (axis + 180.0) % 360.0
        context['axis'] = axis
        return SubtaskStatus.COMPLETED, sub.ctrl.hold(
            sensors, dt, context.get('target_depth', sensors.depth))


class DiveToDepth(Subtask):
    """Reach the task depth at held heading."""
    TIMEOUT = 45.0
    ON_TIMEOUT = 'fail'

    def __init__(self, depth_tolerance: float = 0.12,
                 z_vel_tolerance: float = 0.06):
        super().__init__()
        self.depth_tolerance = depth_tolerance
        self.z_vel_tolerance = z_vel_tolerance
        self.target_depth = 1.0
        self._heading: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._heading = context.get('axis', sensors.heading)
        print(f"INFO: DiveToDepth -> {self.target_depth:.2f} m")

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._heading)
        if (abs(self.target_depth - sensors.depth) < self.depth_tolerance
                and abs(sensors.velocity_z) < self.z_vel_tolerance):
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        return f"{self.name}({self.target_depth:.1f}m)"


class TurnToHeading(Subtask):
    """Compass turn. Target from absolute degrees, relative-to-entry degrees,
    or the task axis (use_axis=True)."""
    TIMEOUT = 35.0
    ON_TIMEOUT = 'fail'

    def __init__(self, absolute_degrees: Optional[float] = None,
                 relative_degrees: Optional[float] = None,
                 use_axis: bool = False,
                 tolerance_degrees: float = 2.5,
                 rate_tolerance: float = 0.05):
        super().__init__()
        if not use_axis and absolute_degrees is None and relative_degrees is None:
            raise ValueError("need absolute_degrees, relative_degrees or use_axis")
        self.absolute = absolute_degrees
        self.relative = relative_degrees
        self.use_axis = use_axis
        self.tol = tolerance_degrees
        self.rate_tol = rate_tolerance
        self.target: Optional[float] = None
        self.target_depth = 1.0

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        if self.use_axis:
            self.target = context.get('axis', sensors.heading)
        elif self.absolute is not None:
            self.target = self.absolute % 360.0
        else:
            self.target = (sensors.heading + self.relative) % 360.0

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self.target)
        if (abs(angle_diff(self.target, sensors.heading)) < self.tol
                and abs(sensors.imu.gyro_z) < self.rate_tol):
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        t = f"{self.target:.0f}" if self.target is not None else "?"
        return f"{self.name}({t} deg)"


class DriveStraight(Subtask):
    """Timed surge on held heading."""
    ON_TIMEOUT = 'complete'

    def __init__(self, duration: float, surge_power: float = 0.5):
        super().__init__()
        self.duration = duration
        self.surge = surge_power
        self.TIMEOUT = duration + 10.0
        self.target_depth = 1.0
        self._heading: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._heading = context.get('axis', sensors.heading)

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._heading,
                             surge=self.surge)
        if self._elapsed >= self.duration:
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds


class SwayStraight(Subtask):
    """Timed open-loop sway on held heading."""
    ON_TIMEOUT = 'complete'

    def __init__(self, duration: float, sway_power: float = 0.5):
        super().__init__()
        self.duration = duration
        self.sway = sway_power
        self.TIMEOUT = duration + 10.0
        self.target_depth = 1.0
        self._heading: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._heading = context.get('axis', sensors.heading)

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._heading,
                             sway=self.sway)
        if self._elapsed >= self.duration:
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        return f"{self.name}({self.sway:+.1f})"


class Stabilize(Subtask):
    """Quiet down at the task depth and held heading. Completes after
    `duration` once rotation rates, roll, depth error and vertical speed are
    all small (the honest stability criteria — no ground-truth velocity)."""
    ON_TIMEOUT = 'complete'

    def __init__(self, duration: float = 2.0, **_legacy):
        super().__init__()
        self.duration = duration
        self.TIMEOUT = duration + 20.0
        self.target_depth = 1.0
        self._heading: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._heading = context.get('axis', sensors.heading)

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._heading)
        if (self._elapsed >= self.duration
                and sub.ctrl.is_settled(sensors, self.target_depth)):
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds


# ---------------------------------------------------------------------------
# search & acquisition
# ---------------------------------------------------------------------------

class AcquireTarget(Subtask):
    """Find the target: hold station and yaw-scan while it is not visible,
    brake and debounce once it is. Scan direction follows the task's
    search_direction."""
    ON_TIMEOUT = 'fail'
    SCAN_RATE = 0.35        # rad/s
    DEBOUNCE_TICKS = 5

    def __init__(self, target_type: str = 'either', timeout: float = 120.0):
        super().__init__()
        self.target_type = target_type.lower()
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._seen = 0

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._seen = 0

    def execute(self, sub, dt, sensors, vision, config, context):
        if _is_visible(vision, self.target_type):
            self._seen += 1
            cmds = sub.ctrl.hold(sensors, dt, self.target_depth)
            if self._seen >= self.DEBOUNCE_TICKS:
                return SubtaskStatus.COMPLETED, cmds
            return SubtaskStatus.RUNNING, cmds
        self._seen = 0
        rate = -self.SCAN_RATE * context.get('search_direction', 1)
        return SubtaskStatus.RUNNING, sub.ctrl.scan(sensors, dt,
                                                    self.target_depth, rate)

    def get_dynamic_name(self, context):
        return f"{self.name}({self.target_type} {self._elapsed:.0f}s)"


# Name kept for compatibility with older mission/task code.
WaitForTargetVisible = AcquireTarget


# ---------------------------------------------------------------------------
# gate leg
# ---------------------------------------------------------------------------

class CenterOnGateHalf(Subtask):
    """Place the chosen half-opening's center at the frame center using pure
    sway with the compass holding the task axis. Completes when the pixel
    error has stayed inside tolerance with a quiet pixel rate.

    rate_tol bounds the RESIDUAL LATERAL VELOCITY at completion (pixel-rate
    is the only lateral-velocity signal the vehicle has) — the pre-roll
    instance uses a tight value so the style roll starts as close to rest as
    physically knowable.

    Range hold: the pair's pixel separation is a range signal; gentle surge
    keeps it near the ~3 m standoff value, so residual closing velocity from
    the previous task can never shrink the FOV until a post falls out of
    frame (that was the return-gate failure mode).

    Blind fallback: sway toward wherever a post is CURRENTLY visible; a
    remembered direction is only trusted briefly (a stale latched sign once
    drove a 40 s runaway into the pool wall), after which the vehicle holds
    station and lets the timeout valve fire.
    """
    ON_TIMEOUT = 'complete'   # valve: never strand the mission on centering
    BLIND_SWAY = 0.25
    SIGN_MEMORY_S = 1.5       # how long the last seen direction stays valid
    SEP_TARGET = 0.95         # pair separation / half-width at ~3 m standoff
    SEP_BAND = 0.28           # completion requires range inside this band
    RANGE_GAIN = 0.6
    TALL_POST_FRAC = 0.4      # single post taller than this frac of frame
                              # height = we are too close (blind regime)

    # W6 2026-07-07: completion tolerances re-derived for the MEASURED sensor
    # noise (compass sigma 1.32 deg at 18.2 Hz -> ~4-5 px of physical pixel
    # jitter at standoff, pixel-rate floor ~0.05-0.08/s). The old 6 px /
    # 0.04/s x12 ticks were tuned on the quiet legacy sim and are
    # unattainable with the real compass — every Center instance burned its
    # full 40 s timeout and the mission degraded to timeout-stumbling.
    def __init__(self, side: str = 'right', tolerance_px: float = 10.0,
                 hold_ticks: int = 8, rate_tol: float = 0.10,
                 timeout: float = 20.0):
        super().__init__()
        self.side = side
        self.tol_px = tolerance_px
        self.hold_ticks = hold_ticks
        self.rate_tol = rate_tol
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._axis: Optional[float] = None
        self._ok = 0
        self._blind = 0.0
        self._sign_age = 1e9
        self._last_err_sign = 0.0

    def on_enter(self, sub, sensors, vision, context):
        if self._depth_override is not None:
            self.target_depth = self._depth_override
        else:
            self.target_depth = context.get('target_depth', sensors.depth)
        self._axis = context.get('axis', sensors.heading)
        self._ok = 0
        self._blind = 0.0
        self._sign_age = 1e9
        self._last_err_sign = 0.0
        self._sep_prev = None
        self._sep_rate = 0.0
        sub.ctrl.reset_pixel_servo()

    def execute(self, sub, dt, sensors, vision, config, context):
        pair = vision.get_gate_pair()
        cam_w = sensors.camera_image.shape[1]
        cam_h = sensors.camera_image.shape[0]
        self._sign_age += dt
        if pair is None:
            self._blind += dt
            self._ok = 0
            posts = vision.get_gate_post_blobs()
            if posts:
                # A single visible post is ambiguous — resolve by RANGE
                # (its apparent height). TALL post: we are too close, the
                # opening fell off the frame edge — back away and sway away
                # from the post. SHORT post: we are at standoff but far off
                # axis, the whole gate sits to one side — sway TOWARD it.
                # (One sign for both regimes ran the vehicle diagonally
                # into the pool corner.) POST blobs only — chasing raw
                # red_blobs steered at the 2026 red maker box (W6).
                blob = max(posts, key=lambda b: b['area'])
                sign = 1.0 if blob['center_x'] > cam_w / 2 else -1.0
                if blob['height'] > self.TALL_POST_FRAC * cam_h:
                    return SubtaskStatus.RUNNING, sub.ctrl.hold(
                        sensors, dt, self.target_depth, self._axis,
                        surge=-0.25, sway=sign * self.BLIND_SWAY)
                return SubtaskStatus.RUNNING, sub.ctrl.hold(
                    sensors, dt, self.target_depth, self._axis,
                    sway=-sign * self.BLIND_SWAY)
            if self._sign_age < self.SIGN_MEMORY_S and self._last_err_sign:
                # Pair just dropped out: chase the last pixel error briefly.
                return SubtaskStatus.RUNNING, sub.ctrl.hold(
                    sensors, dt, self.target_depth, self._axis,
                    sway=-self._last_err_sign * self.BLIND_SWAY)
            if self._blind > 1.0:
                print(f"WARN: CenterOnGateHalf blind {self._blind:.1f}s "
                      f"(no bearing — backing up)")
            # Nothing visible at all: back straight up on the axis — range
            # is the only lever that ever brings the gate back into view.
            return SubtaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth, self._axis, surge=-0.2)
        self._blind = 0.0
        err = _err_norm(gate_half_center_px(pair, self.side), cam_w)
        if abs(err) > 0.02:
            self._last_err_sign = math.copysign(1.0, err)
            self._sign_age = 0.0
        sep = abs(pair[1]['center_x'] - pair[0]['center_x']) / (cam_w / 2.0)
        if self._sep_prev is not None and dt > 0:
            self._sep_rate += 0.3 * ((sep - self._sep_prev) / dt
                                     - self._sep_rate)
        self._sep_prev = sep
        # Caps sized for the quadratic plant: below ~0.12 normalized a
        # corner command produces near-zero force (deadband), so the old
        # ±0.15/0.2 range-hold could not actually move the vehicle.
        surge = float(np.clip((self.SEP_TARGET - sep) * self.RANGE_GAIN,
                              -0.35, 0.25))
        cmds = sub.ctrl.track_pixel_sway(sensors, dt, self.target_depth,
                                         err, self._axis, surge=surge)
        tol = self.tol_px / (cam_w / 2.0)
        # Complete only truly settled on ALL knowable axes: pixel error and
        # rate (lateral), range inside the standoff band and range-rate
        # quiet (longitudinal) — a completion while the range-hold was still
        # surging once sent a roll segment off with forward velocity.
        if (abs(err) < tol and abs(sub.ctrl.pixel_rate()) < self.rate_tol
                and abs(sep - self.SEP_TARGET) < self.SEP_BAND
                and abs(self._sep_rate) < 0.05):
            self._ok += 1
            if self._ok >= self.hold_ticks:
                return SubtaskStatus.COMPLETED, cmds
        else:
            self._ok = 0
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        return f"{self.name}({self.side} ok:{self._ok})"


class DriveThroughGate(Subtask):
    """Drive the committed half-opening: surge with the sway servo trimming
    onto the half-center while both posts are visible; once the gate leaves
    the field of view (close-in), hold the axis and surge a fixed clearance
    time to carry the vehicle through and out the far side."""
    ON_TIMEOUT = 'complete'

    def __init__(self, side: str = 'right', surge_power: float = 0.55,
                 clear_secs: float = 4.0, timeout: float = 45.0):
        super().__init__()
        self.side = side
        self.surge = surge_power
        self.clear_secs = clear_secs
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._axis: Optional[float] = None
        self._seen = False
        self._lost_t: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        if self._depth_override is not None:
            self.target_depth = self._depth_override
        else:
            self.target_depth = context.get('target_depth', sensors.depth)
        self._axis = context.get('axis', sensors.heading)
        self._seen = False
        self._lost_t = None
        sub.ctrl.reset_pixel_servo()

    def execute(self, sub, dt, sensors, vision, config, context):
        pair = vision.get_gate_pair()
        if pair is not None:
            self._seen = True
            self._lost_t = None
            cam_w = sensors.camera_image.shape[1]
            err = _err_norm(gate_half_center_px(pair, self.side), cam_w)
            return SubtaskStatus.RUNNING, sub.ctrl.track_pixel_sway(
                sensors, dt, self.target_depth, err, self._axis,
                surge=self.surge)

        # Gate out of view: either close-in (normal) or never seen (entered
        # early); both end with a timed straight clearance run on the axis.
        if not self._seen and self._elapsed < 4.0:
            return SubtaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth, self._axis, surge=self.surge)
        if self._lost_t is None:
            self._lost_t = self._elapsed
            if not self._seen:
                print("WARN: DriveThroughGate never saw the gate — clearing blind")
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._axis,
                             surge=self.surge)
        if self._elapsed - self._lost_t >= self.clear_secs:
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        phase = 'clear' if self._lost_t is not None else (
            'track' if self._seen else 'entry')
        return f"{self.name}({self.side} {phase})"


class StyleRollSubtask(Subtask):
    """Barrel roll through `degrees` via a RESONANCE PUMP, then level out.

    Water session 1 (2026-07-07) measured submerged righting ~4.6 N*m/rad
    against ~2.6-3.5 N*m of differential-vertical torque at safe power:
    a direct roll STALLS near 50 degrees. But the roll axis is a lightly
    damped pendulum (omega_n ~3 rad/s, zeta ~0.19), so torque applied IN
    PHASE WITH THE ROLL RATE (bang-bang: tau = P*sign(rate)) pumps energy
    at the natural frequency — no hardcoded 0.45 Hz, it self-locks. Once a
    swing crosses ~100 degrees still climbing, righting weakens past
    vertical: switch to constant torque in that direction and carry through
    the top into full rotations. On a plant with enough direct authority
    the pump degenerates into the old direct spin (sign(rate) never flips).

    The accumulated angle integrates imu.gyro_x — the sensor roll channel
    folds at +/-90 degrees and cannot track a full rotation; the folded
    channel is only used by the final leveling, where the true angle is near
    a multiple of 360 and the fold is harmless. Depth stays closed-loop
    through the rotation (the mixer realizes world heave at any roll angle)
    and the compass cascade holds the axis. Times out COMPLETED: style points
    must never strand a mission.
    """
    ON_TIMEOUT = 'complete'
    STOP_LEAD_DEG = 15.0
    COMMIT_RATE = 2.4     # rad/s near upright commits to full rotation:
                          # KE(2.4) ~ 1.7 J vs the ~0.9 J righting-hump
                          # deficit at 0.8 power (offline plant sim W6)
    COMMIT_ANG = 60.0     # deg from upright (mod 360) where KE is honest
    UNCOMMIT_RATE = -0.3  # rad/s against carry dir -> stalled, resume pump
    PUMP_MIN_RATE = 0.05  # rad/s — below this, steer by position not rate

    def __init__(self, degrees: float = 720.0, roll_power: float = 1.00,
                 settle_deg: float = 2.5, settle_rate: float = 0.08,
                 timeout: float = 90.0, target_depth: Optional[float] = None):
        super().__init__()
        self.degrees = degrees
        self._depth_override = target_depth  # None = context/current depth
        self.roll_power = roll_power   # 1.00 since water-2 S9: two 2.5 s
        # 100% roll couples ran with NO brownout, and the +100% pulse did a
        # REAL 360 (gyro-integrated). 100% completes the roll in ~2.5-3.5 s.
        self.settle_deg = settle_deg
        self.settle_rate = settle_rate
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._axis: Optional[float] = None
        self.accum = 0.0
        self.spinning = True
        self._carry_dir = 0.0   # 0 = still pumping; +/-1 = committed

    def on_enter(self, sub, sensors, vision, context):
        if self._depth_override is not None:
            self.target_depth = self._depth_override
        else:
            self.target_depth = context.get('target_depth', sensors.depth)
        self._axis = context.get('axis', sensors.heading)
        self.accum = 0.0
        self.spinning = True
        self._carry_dir = 0.0
        print(f"INFO: StyleRoll {self.degrees:.0f} deg at "
              f"{self.target_depth:.2f} m, axis {self._axis:.1f} (pump mode)")

    def on_exit(self, sub, sensors, vision, context):
        # W7 pacing: if this segment could not finish its rotation (timeout
        # or leveled short), later segments will do no better — mark the
        # mission so they complete instantly instead of burning 45 s each.
        if (self.timed_out
                or abs(self.accum) < abs(self.degrees) - self.STOP_LEAD_DEG):
            context['style_roll_skip'] = True

    def _pump_torque(self, rate: float) -> float:
        """Bang-bang energy pump, robust to the rate crossing zero at the
        swing extremes (and to corrupt zeroed gyro reads): near-zero rate
        steers by position — push AWAY from upright on the return swing."""
        if abs(rate) > self.PUMP_MIN_RATE:
            return math.copysign(self.roll_power, rate)
        if abs(self.accum) > 5.0:
            return -math.copysign(self.roll_power, self.accum)
        return math.copysign(self.roll_power, self.degrees)

    def execute(self, sub, dt, sensors, vision, config, context):
        if context.get('style_roll_skip'):
            print("INFO: StyleRoll skipped (earlier segment could not rotate)")
            return SubtaskStatus.COMPLETED, sub.ctrl.hold(
                sensors, dt, self.target_depth, self._axis)
        rate = sensors.imu.gyro_x
        self.accum += math.degrees(rate) * dt

        if self.spinning:
            ang_up = abs(self.accum) % 360.0   # distance past the last upright
            near_upright = (ang_up < self.COMMIT_ANG
                            or ang_up > 360.0 - self.COMMIT_ANG)
            if self._carry_dir == 0.0:
                # Pump phase. Commit to rotation only with real kinetic
                # energy in the bank — rate threshold NEAR UPRIGHT, where
                # measured KE is honest (at the swing extremes rate is
                # always small). Offline plant sim: with linear-model roll
                # drag this completes 720 in ~11 s; with the (pessimistic,
                # unverified) quadratic model it pumps ~70 deg swings and
                # times out COMPLETED — mission-safe either way. S9 decides.
                if abs(rate) >= self.COMMIT_RATE and near_upright:
                    self._carry_dir = math.copysign(1.0, rate)
                    print(f"INFO: StyleRoll pump -> carry at "
                          f"{self.accum:+.0f} deg, {rate:+.1f} rad/s")
            elif rate * self._carry_dir < self.UNCOMMIT_RATE:
                # Stalled against the righting hump and falling back:
                # abandon the attempt and pump the return swing instead.
                print(f"INFO: StyleRoll carry stalled at {self.accum:+.0f} "
                      f"deg — back to pump")
                self._carry_dir = 0.0
            if self._carry_dir != 0.0:
                # Rotations score either direction; chase |degrees| in the
                # direction the pump actually launched.
                if abs(self.accum) >= abs(self.degrees) - self.STOP_LEAD_DEG:
                    self.spinning = False
                else:
                    return SubtaskStatus.RUNNING, sub.ctrl.roll_spin(
                        sensors, dt, self.target_depth,
                        self._carry_dir * self.roll_power,
                        self._axis, self.accum)
            else:
                return SubtaskStatus.RUNNING, sub.ctrl.roll_spin(
                    sensors, dt, self.target_depth,
                    self._pump_torque(rate), self._axis, self.accum)

        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._axis)
        if (abs(angle_diff(0.0, sensors.roll)) < self.settle_deg
                and abs(sensors.imu.gyro_x) < self.settle_rate):
            print(f"INFO: StyleRoll done: {self.accum:.0f} deg, "
                  f"leveled at {sensors.roll:+.1f}")
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        phase = ('carry' if self._carry_dir else 'pump') if self.spinning else 'level'
        return f"{self.name}({self.accum:+.0f}/{self.degrees:.0f} {phase})"


# ---------------------------------------------------------------------------
# generic vision-drive (kept for orbit course & compatibility)
# ---------------------------------------------------------------------------

class DriveUntilTargetLost(Subtask):
    """Surge on the held axis until a previously-visible target is lost.
    Fails only if the target is never seen within the acquire window."""
    ON_TIMEOUT = 'complete'

    def __init__(self, surge_power: float = 0.5, target_type: str = 'gate',
                 acquire_timeout: float = 10.0, timeout: float = 60.0):
        super().__init__()
        self.surge = surge_power
        self.target_type = target_type
        self.acquire_timeout = acquire_timeout
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._axis: Optional[float] = None
        self._seen = False

    def on_enter(self, sub, sensors, vision, context):
        if self._depth_override is not None:
            self.target_depth = self._depth_override
        else:
            self.target_depth = context.get('target_depth', sensors.depth)
        self._axis = context.get('axis', sensors.heading)
        self._seen = False

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._axis,
                             surge=self.surge)
        if _is_visible(vision, self.target_type):
            self._seen = True
            return SubtaskStatus.RUNNING, cmds
        if self._seen:
            return SubtaskStatus.COMPLETED, cmds
        if self._elapsed > self.acquire_timeout:
            print(f"ERROR: DriveUntilTargetLost never saw '{self.target_type}'")
            return SubtaskStatus.FAILED, cmds
        return SubtaskStatus.RUNNING, cmds


# Forward variant is the same behavior at a different default power.
class DriveUntilTargetLostForward(DriveUntilTargetLost):
    def __init__(self, surge_power: float = 0.4, target_type: str = 'pole'):
        super().__init__(surge_power=surge_power, target_type=target_type)


class SwayUntilTargetLost(Subtask):
    """Sway on the held axis until the target leaves the frame."""
    ON_TIMEOUT = 'complete'

    def __init__(self, sway_power: float = -0.3, target_type: str = 'pole',
                 timeout: float = 45.0):
        super().__init__()
        self.sway = sway_power
        self.target_type = target_type
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._axis: Optional[float] = None

    def on_enter(self, sub, sensors, vision, context):
        if self._depth_override is not None:
            self.target_depth = self._depth_override
        else:
            self.target_depth = context.get('target_depth', sensors.depth)
        self._axis = context.get('axis', sensors.heading)

    def execute(self, sub, dt, sensors, vision, config, context):
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._axis,
                             sway=self.sway)
        if not _is_visible(vision, self.target_type):
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds

    def get_dynamic_name(self, context):
        return f"{self.name}({self.sway:+.1f})"


class AlignToObjectX(Subtask):
    """Point the nose at the target (pixel-yaw). Used on the orbit course
    where the vehicle must face the marker; the gate leg centers by sway
    instead (see CenterOnGateHalf)."""
    ON_TIMEOUT = 'fail'

    def __init__(self, target_x_fraction: float = 0.5, tolerance_px: int = 12,
                 timeout: float = 40.0, **_legacy):
        super().__init__()
        self.frac = target_x_fraction
        self.tol_px = tolerance_px
        self.TIMEOUT = timeout
        self.target_depth = 1.0

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        sub.ctrl.reset_pixel_servo()

    def execute(self, sub, dt, sensors, vision, config, context):
        cx = _target_center_x(vision)
        if cx is None:
            return SubtaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth)
        cam_w = sensors.camera_image.shape[1]
        err_px = cx - cam_w * self.frac
        cmds = sub.ctrl.track_pixel_yaw(sensors, dt, self.target_depth,
                                        err_px / (cam_w / 2.0))
        if abs(err_px) < self.tol_px and abs(sensors.imu.gyro_z) < 0.05:
            context['axis'] = sensors.heading   # facing the target now
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds


class ApproachAndCenterObject(Subtask):
    """Close on the pole to a target apparent height while keeping the nose
    on it (orbit course)."""
    ON_TIMEOUT = 'fail'

    def __init__(self, height_threshold_px: int,
                 target_x_fraction: float = 0.5, surge_p_gain: float = 0.1,
                 height_tolerance_px: int = 10, lost_timeout: float = 2.5,
                 timeout: float = 90.0, **_legacy):
        super().__init__()
        self.height_px = height_threshold_px
        self.frac = target_x_fraction
        self.surge_p = surge_p_gain
        self.height_tol = height_tolerance_px
        self.lost_timeout = lost_timeout
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._lost = 0.0

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._lost = 0.0
        sub.ctrl.reset_pixel_servo()

    def execute(self, sub, dt, sensors, vision, config, context):
        pole = vision.get_best_pole()
        if pole is None:
            self._lost += dt
            if self._lost > self.lost_timeout:
                print("ERROR: ApproachAndCenterObject lost the pole")
                return SubtaskStatus.FAILED, sub.ctrl.hold(
                    sensors, dt, self.target_depth)
            return SubtaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth)
        self._lost = 0.0
        cam_w = sensors.camera_image.shape[1]
        err_px = pole['center_x'] - cam_w * self.frac
        h_err = self.height_px - pole['height']
        surge = float(np.clip(h_err * self.surge_p, -0.4, 0.5))
        cmds = sub.ctrl.track_pixel_yaw(sensors, dt, self.target_depth,
                                        err_px / (cam_w / 2.0), surge=surge)
        if abs(h_err) < self.height_tol and abs(err_px) < 20:
            context['axis'] = sensors.heading
            return SubtaskStatus.COMPLETED, cmds
        return SubtaskStatus.RUNNING, cmds


class DynamicOrbitPole(Subtask):
    """Orbit the pole (constant sway + pixel-yaw pointing + width-PID range
    hold) until the gate appears to the pole's right (orbit course exit
    condition)."""
    ON_TIMEOUT = 'complete'   # valve: proceed to the return gate regardless

    def __init__(self, target_x_fraction: float = 0.5,
                 sway_power: float = -0.3,
                 target_pole_width_fraction: float = 0.06,
                 orbit_width_p_gain: float = 0.03,
                 orbit_width_i_gain: float = 0.01,
                 orbit_width_d_gain: float = 0.05,
                 orbit_width_i_clamp: float = 0.3,
                 lost_timeout: float = 3.0,
                 min_orbit_time: float = 5.0,
                 timeout: float = 150.0, **_legacy):
        super().__init__()
        self.frac = target_x_fraction
        self.sway = sway_power
        self.width_frac = target_pole_width_fraction
        self.wp, self.wi, self.wd = (orbit_width_p_gain, orbit_width_i_gain,
                                     orbit_width_d_gain)
        self.wi_clamp = orbit_width_i_clamp
        self.lost_timeout = lost_timeout
        self.min_orbit_time = min_orbit_time
        self.TIMEOUT = timeout
        self.target_depth = 1.0
        self._lost = 0.0
        self._orbit_t = 0.0
        self._w_i = 0.0
        self._w_err_prev = 0.0

    def on_enter(self, sub, sensors, vision, context):
        self.target_depth = context.get('target_depth', sensors.depth)
        self._lost = 0.0
        self._orbit_t = 0.0
        self._w_i = 0.0
        self._w_err_prev = 0.0
        sub.ctrl.reset_pixel_servo()

    def execute(self, sub, dt, sensors, vision, config, context):
        pole = vision.get_best_pole()
        gate_x = vision.get_gate_center_x() if vision.is_gate_visible() else None

        if (pole is not None and gate_x is not None
                and gate_x > pole['center_x']
                and self._orbit_t >= self.min_orbit_time):
            print("INFO: orbit exit — gate right of pole")
            context['axis'] = sensors.heading
            return SubtaskStatus.COMPLETED, sub.ctrl.hold(
                sensors, dt, self.target_depth)

        if pole is None:
            self._lost += dt
            if self._lost > self.lost_timeout:
                print("ERROR: DynamicOrbitPole lost the pole")
                return SubtaskStatus.FAILED, sub.ctrl.hold(
                    sensors, dt, self.target_depth)
            return SubtaskStatus.RUNNING, sub.ctrl.scan(
                sensors, dt, self.target_depth, -0.2)

        self._lost = 0.0
        self._orbit_t += dt
        cam_w = sensors.camera_image.shape[1]
        err_px = pole['center_x'] - cam_w * self.frac
        w_err = cam_w * self.width_frac - pole['width']
        self._w_i = float(np.clip(self._w_i + w_err * dt,
                                  -self.wi_clamp, self.wi_clamp))
        w_rate = (w_err - self._w_err_prev) / dt if dt > 0 else 0.0
        self._w_err_prev = w_err
        surge = float(np.clip(w_err * self.wp + self._w_i * self.wi
                              + w_rate * self.wd, -0.5, 0.5))
        return SubtaskStatus.RUNNING, sub.ctrl.track_pixel_yaw(
            sensors, dt, self.target_depth, err_px / (cam_w / 2.0),
            surge=surge, sway=self.sway)

    def get_dynamic_name(self, context):
        return f"{self.name}(orbit {self._orbit_t:.0f}s)"
