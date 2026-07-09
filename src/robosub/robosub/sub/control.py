#!/usr/bin/env python3
"""Single control layer for the submarine.

Every task expresses intent through MotionController; this module is the ONLY
place where sensor feedback becomes ThrusterCommands. One gain set, loaded
from config/pid_params.yaml, all angular quantities in radians and rad/s.

Sensor contract (identical on simulator and vehicle): camera image, depth
(fused on hardware), vertical velocity, heading, roll, gyro rates. The real
vehicle has NO lateral velocity sensing, so no law in this module may read
sensors.velocity_x / velocity_y — lateral damping comes from the pixel-rate
of the visual target instead.

Control laws:
  heading   cascade: angle error -> capped desired yaw rate -> rate loop on
            gyro_z. Converges from any error without a dead zone.
  depth     PID on fused depth + a constant buoyancy feedforward, so the
            integrator only carries residual trim and the vertical thrust
            budget is predictable for the heave/roll arbitration.
  roll      PD on roll angle (radians) against gyro_x.
  pixel     two visual servos: pixel-x -> yaw (pointing) and pixel-x -> sway
            (translation at held heading). The sway servo keeps a filtered
            pixel-rate estimate as its damping term.

Mixing: body-frame surge/sway/yaw on the four 45-degree corner thrusters;
world-frame heave + roll torque on the two vertical thrusters, using the
exact inverse of the simulator's roll rotation so commanded world heave and
sway are realized at any roll angle. Heave has priority over roll for the
shared vertical budget.
"""
import math
import os
from typing import Optional, Tuple

import numpy as np
import yaml

from robosub.sub.data_structures import ThrusterCommands, SensorSuite
from robosub.sub.utils import angle_diff

_DEFAULTS = {
    # --- heading cascade ---
    'yaw_p':          1.6,    # heading error (rad) -> desired yaw rate (rad/s)
    'max_yaw_rate':   0.5,    # rad/s cap on desired rate
    'yaw_rate_p':     2.5,    # rate error (rad/s) -> yaw command
    # --- visual servos (pixel errors normalized by half image width) ---
    'pixel_yaw_p':    0.7,
    'pixel_yaw_d':    2.0,    # gyro_z damping in pixel-yaw mode
    'pixel_sway_p':   1.1,
    'pixel_sway_d':   2.6,    # filtered pixel-rate (1/s) -> sway damping
    'pixel_rate_alpha': 0.3,  # LPF coefficient for the pixel-rate estimate
    # --- vertical ---
    'depth_p':        2.5,
    'depth_i':        0.12,
    'depth_i_clamp':  1.2,
    'depth_d':        2.0,    # fused vertical velocity (m/s)
    'buoyancy_ff':    0.61,   # steady heave that cancels net buoyancy
    # --- roll ---
    'roll_p':         0.6,    # roll error (rad) -> cmd. WATER 1st test
                             # (2026-07-09): 2.3 saturated 97%% of the time ->
                             # bang-bang full diff (>righting) -> +-87 deg limit
                             # cycle, near-tip. Gentle P + rely on natural righting.
    'roll_d':         3.0,    # gyro_x (rad/s) -- more damping (sub is
                             # very underdamped, zeta~0.19); on the ACCURATE raw
                             # gyro, not the lagged attitude, so it is safe.
    'roll_authority_max': 0.4,  # HARD cap on roll cmd: 0.4*full ~= 2.2 N*m <
                             # 3.3 N*m righting, so control can never out-torque
                             # the hull and flip it (the water-1 near-tip mode).
    # --- vision thresholds (consumed by Submarine, kept in one file) ---
    'min_pixels_for_detection': 20,
    'min_gate_pixels': 50,
}

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'config', 'pid_params.yaml'))

_COS45 = 0.7071


def load_params() -> dict:
    params = dict(_DEFAULTS)
    if os.path.isfile(_CONFIG_FILE):
        with open(_CONFIG_FILE) as f:
            overrides = yaml.safe_load(f) or {}
        unknown = [k for k in overrides if k not in params]
        params.update({k: v for k, v in overrides.items() if k in params})
        print(f"INFO: control params loaded from {_CONFIG_FILE}"
              + (f" (ignored unknown keys: {unknown})" if unknown else ""))
    else:
        print(f"WARN: {_CONFIG_FILE} not found, using built-in defaults")
    return params


class MotionController:

    def __init__(self, params: Optional[dict] = None):
        self.p = params or load_params()
        self.reset()

    def reset(self):
        self._depth_i = 0.0
        self._depth_target_prev: Optional[float] = None
        self.reset_pixel_servo()

    # ------------------------------------------------------------------
    # Pixel-rate state (single lateral/pointing servo channel)
    # ------------------------------------------------------------------

    def reset_pixel_servo(self):
        self._px_err_prev: Optional[float] = None
        self._px_rate = 0.0

    def _update_pixel_rate(self, err_norm: float, dt: float) -> float:
        if self._px_err_prev is not None and dt > 0:
            raw = (err_norm - self._px_err_prev) / dt
            a = self.p['pixel_rate_alpha']
            self._px_rate += a * (raw - self._px_rate)
        self._px_err_prev = err_norm
        return self._px_rate

    def pixel_rate(self) -> float:
        """Filtered normalized pixel-error rate (1/s) of the active servo."""
        return self._px_rate

    # ------------------------------------------------------------------
    # Axis laws
    # ------------------------------------------------------------------

    def _yaw_hold(self, sensors: SensorSuite, heading_deg: float) -> float:
        err = math.radians(angle_diff(heading_deg, sensors.heading))
        rate_des = float(np.clip(err * self.p['yaw_p'],
                                 -self.p['max_yaw_rate'], self.p['max_yaw_rate']))
        return float(np.clip((rate_des - sensors.imu.gyro_z) * self.p['yaw_rate_p'],
                             -1.0, 1.0))

    def _yaw_rate(self, sensors: SensorSuite, rate: float) -> float:
        rate = float(np.clip(rate, -self.p['max_yaw_rate'], self.p['max_yaw_rate']))
        return float(np.clip((rate - sensors.imu.gyro_z) * self.p['yaw_rate_p'],
                             -1.0, 1.0))

    def _heave(self, sensors: SensorSuite, depth_target: float, dt: float) -> float:
        if (self._depth_target_prev is not None
                and abs(depth_target - self._depth_target_prev) > 0.2):
            self._depth_i = 0.0
        self._depth_target_prev = depth_target
        err = depth_target - sensors.depth
        self._depth_i = float(np.clip(self._depth_i + err * dt,
                                      -self.p['depth_i_clamp'],
                                      self.p['depth_i_clamp']))
        return float(np.clip(
            self.p['buoyancy_ff']
            + err * self.p['depth_p']
            + self._depth_i * self.p['depth_i']
            - sensors.velocity_z * self.p['depth_d'],
            -1.0, 1.0))

    def _roll_level(self, sensors: SensorSuite, target_deg: float = 0.0) -> float:
        err = math.radians(angle_diff(target_deg, sensors.roll))
        cap = self.p.get('roll_authority_max', 1.0)
        return float(np.clip(err * self.p['roll_p']
                             - sensors.imu.gyro_x * self.p['roll_d'], -cap, cap))

    # ------------------------------------------------------------------
    # Public command builders (each returns complete ThrusterCommands)
    # ------------------------------------------------------------------

    def hold(self, sensors: SensorSuite, dt: float, depth: float,
             heading: Optional[float] = None,
             surge: float = 0.0, sway: float = 0.0) -> ThrusterCommands:
        """Hold depth + heading (current heading if None), with optional
        open-loop surge/sway power."""
        hdg = sensors.heading if heading is None else heading
        return self._mix(surge, sway,
                         self._yaw_hold(sensors, hdg),
                         self._heave(sensors, depth, dt),
                         self._roll_level(sensors),
                         sensors.roll)

    def scan(self, sensors: SensorSuite, dt: float, depth: float,
             yaw_rate: float, surge: float = 0.0) -> ThrusterCommands:
        """Constant-rate yaw scan while holding depth."""
        return self._mix(surge, 0.0,
                         self._yaw_rate(sensors, yaw_rate),
                         self._heave(sensors, depth, dt),
                         self._roll_level(sensors),
                         sensors.roll)

    def track_pixel_yaw(self, sensors: SensorSuite, dt: float, depth: float,
                        err_norm: float, surge: float = 0.0,
                        sway: float = 0.0) -> ThrusterCommands:
        """Point the nose at a visual target: pixel error -> yaw. Optional
        open-loop sway (used by the orbit maneuver)."""
        self._update_pixel_rate(err_norm, dt)
        yaw = float(np.clip(-err_norm * self.p['pixel_yaw_p']
                            - sensors.imu.gyro_z * self.p['pixel_yaw_d'],
                            -1.0, 1.0))
        return self._mix(surge, sway, yaw,
                         self._heave(sensors, depth, dt),
                         self._roll_level(sensors),
                         sensors.roll)

    def track_pixel_sway(self, sensors: SensorSuite, dt: float, depth: float,
                         err_norm: float, heading: float,
                         surge: float = 0.0) -> ThrusterCommands:
        """Translate laterally to null a pixel error while the compass holds
        heading — vision owns lateral position, never heading. Damped by the
        filtered pixel rate (the honest lateral-velocity signal)."""
        rate = self._update_pixel_rate(err_norm, dt)
        sway = float(np.clip(-err_norm * self.p['pixel_sway_p']
                             - rate * self.p['pixel_sway_d'], -1.0, 1.0))
        return self._mix(surge, sway,
                         self._yaw_hold(sensors, heading),
                         self._heave(sensors, depth, dt),
                         self._roll_level(sensors),
                         sensors.roll)

    def roll_spin(self, sensors: SensorSuite, dt: float, depth: float,
                  roll_power: float, heading: float,
                  unwrapped_roll_deg: float) -> ThrusterCommands:
        """Continuous roll (style maneuver): full available roll torque while
        world heave keeps the depth loop closed through the rotation. The
        caller supplies its gyro-integrated unwrapped angle — the folded
        sensor roll channel is wrong past +/-90 degrees."""
        return self._mix(0.0, 0.0,
                         self._yaw_hold(sensors, heading),
                         self._heave(sensors, depth, dt),
                         float(np.clip(roll_power, -1.0, 1.0)),
                         unwrapped_roll_deg)

    # ------------------------------------------------------------------
    # Settledness checks (shared completion criteria)
    # ------------------------------------------------------------------

    def is_settled(self, sensors: SensorSuite, depth_target: float,
                   depth_tol: float = 0.12, rate_tol: float = 0.06,
                   roll_tol_deg: float = 4.0) -> bool:
        """Quiescent by the signals the real vehicle can sense: rotation
        rates, roll angle, depth error and vertical speed."""
        return (abs(sensors.imu.gyro_z) < rate_tol
                and abs(sensors.imu.gyro_x) < rate_tol
                and abs(angle_diff(0.0, sensors.roll)) < roll_tol_deg
                and abs(depth_target - sensors.depth) < depth_tol
                and abs(sensors.velocity_z) < 0.08)

    # ------------------------------------------------------------------
    # Mixer
    # ------------------------------------------------------------------

    def _mix(self, surge: float, sway: float, yaw: float,
             world_heave: float, roll_cmd: float,
             roll_deg: float) -> ThrusterCommands:
        """5-axis -> 6 thrusters.

        (sway, heave) are WORLD-frame intents. The simulator (and the water)
        rotate body-frame vertical/lateral thrust through the vehicle's roll,
        so apply the exact inverse rotation here:
            sway_b  =  S*cos(r) + W*sin(r)
            heave_b = -S*sin(r) + W*cos(r)
        Heave keeps priority over roll on the shared vertical pair; roll uses
        whatever budget remains (which grows to the full budget near +/-90
        degrees, where vertical thrust has no world-vertical component).
        """
        r = math.radians(roll_deg)
        S = float(np.clip(sway, -1.0, 1.0))
        W = float(np.clip(world_heave, -1.0, 1.0))
        sway_b = S * math.cos(r) + W * math.sin(r)
        heave_b = float(np.clip(-S * math.sin(r) + W * math.cos(r), -1.0, 1.0))

        roll_budget = max(0.0, 1.0 - abs(heave_b))
        roll_c = float(np.clip(roll_cmd, -roll_budget, roll_budget))

        s = float(np.clip(surge, -1.0, 1.0)) * _COS45
        w = float(np.clip(sway_b, -1.0, 1.0)) * _COS45
        cmds = ThrusterCommands(
            hfl=s + w + yaw,
            hfr=s - w - yaw,
            hal=s - w + yaw,
            har=s + w - yaw,
            vp=heave_b + roll_c,
            vs=heave_b - roll_c,
        )
        h_max = max(1.0, abs(cmds.hfl), abs(cmds.hfr), abs(cmds.hal), abs(cmds.har))
        if h_max > 1.0:
            cmds.hfl /= h_max
            cmds.hfr /= h_max
            cmds.hal /= h_max
            cmds.har /= h_max
        return cmds
