#!/usr/bin/env python3
"""
Implementations of common, reusable Subtasks, using context.
Subtasks now expect a Vision object and hold depth.
"""
import math
from typing import Tuple, List, Optional, Dict, Any
import numpy as np

# Absolute import for the base class
from robosub.sub.tasks.subtask_base import Subtask, SubtaskStatus
# --- Import Vision class ---
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
# ---
from robosub.sub.config import SimulationConfig
from robosub.sub.utils import angle_diff
# --- NO IMPORT from task_base here ---

# --- DiveToDepth Subtask ---
# ... (DiveToDepth code remains the same) ...
class DiveToDepth(Subtask):
    def __init__(self, depth_tolerance: float = 0.15, z_vel_tolerance: float = 0.05):
        self.depth_tolerance = depth_tolerance
        self.z_vel_tolerance = z_vel_tolerance
        self.target_depth: float = 1.0
        self.target_set = False
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.target_depth = context.get('target_depth', sensors.depth)
        self.target_set = True
        print(f"INFO: DiveToDepth starting. Target: {self.target_depth:.2f}m")
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if not self.target_set: self.on_enter(sub, sensors, vision_data, context)
        depth_error = self.target_depth - sensors.depth
        if (abs(depth_error) < self.depth_tolerance and abs(sensors.velocity_z) < self.z_vel_tolerance):
            print("INFO: DiveToDepth complete."); return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        return SubtaskStatus.RUNNING, sub._get_damping_commands(sensors)
    def get_dynamic_name(self, context: Dict[str, Any]) -> str: return f"{self.name}({self.target_depth:.1f}m)"

# --- SwayStraight Subtask ---
# ... (SwayStraight code remains the same) ...
class SwayStraight(Subtask):
    def __init__(self, duration: float, sway_power: float = 0.5):
        self.duration = duration; self.sway_power = sway_power; self.timer = 0.0
        self.heading_to_hold = None; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.timer = self.duration; self.heading_to_hold = context.get('initial_heading', sensors.heading)
        self.target_depth = context.get('target_depth', sensors.depth)
        print(f"INFO: SwayStraight starting. Power: {self.sway_power}, Duration: {self.duration}")
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.timer <= 0: return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        self.timer -= dt
        if self.heading_to_hold is None: self.heading_to_hold = sensors.heading
        commands = sub.get_heading_commands(sensors, self.heading_to_hold, surge_power=0.0, sway_power=self.sway_power, target_depth=self.target_depth)
        return SubtaskStatus.RUNNING, commands
    def get_dynamic_name(self, context: Dict[str, Any]) -> str: return f"{self.name}({self.sway_power:+.1f})"

# --- DriveUntilTargetLostForward Subtask ---
# ... (DriveUntilTargetLostForward code remains the same) ...
class DriveUntilTargetLostForward(Subtask):
    def __init__(self, surge_power: float = 0.4, target_type: str = 'pole'):
        if target_type not in ['gate', 'pole', 'either']: raise ValueError("target_type must be 'gate', 'pole', or 'either'")
        self.surge_power = surge_power; self.target_type = target_type
        self.heading_to_hold: Optional[float] = None; self.target_was_visible = False; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision: Vision, context: Dict[str, Any]):
        self.heading_to_hold = context.get('initial_heading', sensors.heading)
        self.target_depth = context.get('target_depth', sensors.depth)
        if self.target_type == 'gate': self.target_was_visible = vision.is_gate_visible()
        elif self.target_type == 'pole': self.target_was_visible = vision.is_pole_visible()
        else: self.target_was_visible = vision.is_gate_visible() or vision.is_pole_visible()
        if not self.target_was_visible: print(f"WARN: DriveUntilTargetLostForward starting but target '{self.target_type}' not visible!")
        else: print(f"INFO: DriveUntilTargetLostForward starting. Target '{self.target_type}' visible.")
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.heading_to_hold is None: self.heading_to_hold = sensors.heading
        currently_visible = False
        if self.target_type == 'gate': currently_visible = vision.is_gate_visible()
        elif self.target_type == 'pole': currently_visible = vision.is_pole_visible()
        else: currently_visible = vision.is_gate_visible() or vision.is_pole_visible()
        if currently_visible:
            self.target_was_visible = True
            commands = sub.get_heading_commands(sensors, self.heading_to_hold, self.surge_power, target_depth=self.target_depth)
            return SubtaskStatus.RUNNING, commands
        else:
            if self.target_was_visible:
                print(f"INFO: DriveUntilTargetLostForward complete. Target '{self.target_type}' lost.")
                context['initial_heading'] = sensors.heading; return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
            else:
                print(f"ERROR: DriveUntilTargetLostForward failed. Target '{self.target_type}' never visible.")
                return SubtaskStatus.FAILED, sub._get_damping_commands(sensors)

# --- MODIFIED DynamicOrbitPole Subtask ---
class DynamicOrbitPole(Subtask):
    """
    Orbits the pole by applying constant sway and using yaw control
    to keep the pole at a target X fraction. Uses PID surge control based
    on apparent WIDTH to maintain distance.
    Completes when the gate is visible AND the pole is >= 80% screen width.
    """
    def __init__(self,
                 target_x_fraction: float = 0.5,
                 sway_power: float = -0.3,
                 yaw_gain: float = 1.5,
                 # --- CHANGED to WIDTH ---
                 target_pole_width_fraction: float = 0.12, # Target width as fraction of image width
                 orbit_width_p_gain: float = 0.05, # P gain for width error -> surge (NEEDS TUNING)
                 orbit_width_i_gain: float = 0.01, # I gain (NEEDS TUNING)
                 orbit_width_d_gain: float = 0.05, # D gain (NEEDS TUNING)
                 orbit_width_i_clamp: float = 0.3, # Max integral contribution
                 # ---
                 lost_timeout: float = 3.0,
                 local_search_yaw: float = -0.2,
                 min_orbit_time: float = 5.0):

        self.target_x_fraction = target_x_fraction
        self.sway_power = sway_power
        self.yaw_gain = yaw_gain
        self.min_orbit_time = min_orbit_time
        # Width PID parameters
        self.target_pole_width_fraction = target_pole_width_fraction
        self.width_p_gain = orbit_width_p_gain
        self.width_i_gain = orbit_width_i_gain
        self.width_d_gain = orbit_width_d_gain
        self.width_i_clamp = orbit_width_i_clamp
        # ---
        self.lost_timeout = lost_timeout
        self.local_search_yaw = local_search_yaw
        self.target_depth: float = 0.1
        self.time_since_target_lost = 0.0
        self.orbit_time = 0.0         # Time spent actively orbiting
        self.integral_width_err = 0.0 # Width integral term
        self.last_width_error = 0.0   # For D term calculation

    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.target_depth = context.get('target_depth', sensors.depth)
        self.time_since_target_lost = 0.0
        self.integral_width_err = 0.0 # Reset integral
        self.last_width_error = 0.0
        # Get initial width error
        pole_blob = vision_data.get_best_pole()
        if pole_blob:
            current_width = pole_blob['width']
            self.last_width_error = 0.0  # will be computed properly in execute()
        else:
            self.last_width_error = 0.0

        self.orbit_time = 0.0
        print(f"INFO: DynamicOrbitPole starting. Target X: {self.target_x_fraction*100:.0f}%, Sway: {self.sway_power}, Target Width: {self.target_pole_width_fraction*100:.1f}% of image, Min orbit time: {self.min_orbit_time:.1f}s")

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        pole_visible = vision_data.is_pole_visible()
        pole_blob = vision_data.get_best_pole() # Get the full blob data
        pole_center_x = pole_blob['center_x'] if pole_blob else None
        current_width = pole_blob['width'] if pole_blob else 0.0
        gate_visible = vision_data.is_gate_visible()

        cam_w = sensors.camera_image.shape[1]
        target_pole_x_px = cam_w * self.target_x_fraction
        target_width_px = cam_w * self.target_pole_width_fraction

        # --- Completion Check: Gate Visible AND Gate Right of Pole? ---
        gate_right_of_pole = False
        gate_center_x = vision_data.get_gate_center_x()
        if gate_visible and pole_visible and gate_center_x is not None and pole_center_x is not None:
            gate_right_of_pole = (gate_center_x > pole_center_x)

        # Require minimum orbit time before completing to prevent immediate exit
        if gate_visible and pole_visible and gate_right_of_pole and self.orbit_time < self.min_orbit_time:
            gate_right_of_pole = False  # keep orbiting

        # Require gate visibility AND it being right of the pole
        if gate_visible and pole_visible and gate_right_of_pole:
            print(f"INFO: DynamicOrbitPole found gate ({gate_visible}) right of pole ({gate_right_of_pole}). Completing.")
            context['initial_heading'] = sensors.heading # Store heading
            return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)

        if pole_visible and pole_center_x is not None:
            # --- Pole is visible: Execute Orbit Control ---
            self.time_since_target_lost = 0.0
            self.orbit_time += dt

            pixel_error_x = pole_center_x - target_pole_x_px
            yaw_cmd = float(np.clip(
                -(pixel_error_x / (cam_w / 2)) * self.yaw_gain - sensors.imu.gyro_z * sub.YAW_D_GAIN,
                -1.0, 1.0
            ))

            sway_cmd = self.sway_power

            # --- Surge PID Controller based on WIDTH ---
            width_error = target_width_px - current_width
            # Update integral term
            self.integral_width_err += width_error * dt
            self.integral_width_err = np.clip(self.integral_width_err, -self.width_i_clamp, self.width_i_clamp)
            # Calculate derivative of error
            error_rate = (width_error - self.last_width_error) / dt if dt > 0 else 0.0
            self.last_width_error = width_error # Store for next iteration
            # Calculate PID terms
            surge_p = width_error * self.width_p_gain
            surge_i = self.integral_width_err * self.width_i_gain
            surge_d = error_rate * self.width_d_gain # D acts on rate of change of error
            # Combine terms (Positive error = too far = positive surge)
            surge_cmd = surge_p + surge_i + surge_d
            surge_cmd = np.clip(surge_cmd, -0.5, 0.5)
            # ---

            # Hold Depth
            heave_cmd, roll_cmd = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0)

            # Mix commands
            commands = sub._mix_and_normalize_commands(surge_cmd, sway_cmd, yaw_cmd, heave_cmd, roll_cmd)
            return SubtaskStatus.RUNNING, commands
        else:
            # --- Pole is LOST: Local Search or Fail ---
            self.time_since_target_lost += dt
            if self.time_since_target_lost > self.lost_timeout:
                print(f"ERROR: DynamicOrbitPole failed - pole lost for > {self.lost_timeout}s")
                return SubtaskStatus.FAILED, sub._get_damping_commands(sensors)
            # Spin to search locally, stop surge/sway
            yaw_cmd = self.local_search_yaw
            sway_cmd = 0.0
            surge_cmd = 0.0
            self.integral_width_err = 0.0 # Reset integral
            self.last_width_error = 0.0
            # Hold Depth
            heave_cmd, roll_cmd = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0)
            commands = sub._mix_and_normalize_commands(surge_cmd, sway_cmd, yaw_cmd, heave_cmd, roll_cmd)
            return SubtaskStatus.RUNNING, commands

    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        search_indicator = f" Searching! ({self.time_since_target_lost:.1f}s)" if self.time_since_target_lost > 0 else ""
        return f"{self.name}({self.target_x_fraction*100:.0f}%){search_indicator}"

# --- SwayUntilTargetLost Subtask ---
# ... (SwayUntilTargetLost code remains the same) ...
class SwayUntilTargetLost(Subtask):
    """
    Sways straight (holding heading and depth) until the specified
    vision target (default='pole') is lost.
    """
    def __init__(self, sway_power: float = -0.3, target_type: str = 'pole'):
        if target_type not in ['gate', 'pole', 'either']:
             raise ValueError("target_type must be 'gate', 'pole', or 'either'")
        self.sway_power = sway_power # Negative = Left
        self.target_type = target_type
        self.heading_to_hold: Optional[float] = None
        self.target_was_visible = False
        self.target_depth: float = 0.1

    def on_enter(self, sub: 'Submarine', sensors: SensorSuite,
                 vision: Vision,
                 context: Dict[str, Any]):
        self.heading_to_hold = context.get('initial_heading', sensors.heading)
        self.target_depth = context.get('target_depth', sensors.depth)
        if self.target_type == 'gate':
             self.target_was_visible = vision.is_gate_visible()
        elif self.target_type == 'pole':
             self.target_was_visible = vision.is_pole_visible()
        else: # either
             self.target_was_visible = vision.is_gate_visible() or vision.is_pole_visible()
        if not self.target_was_visible:
            print(f"WARN: SwayUntilTargetLost starting but target '{self.target_type}' not visible!")
        else:
            print(f"INFO: SwayUntilTargetLost starting. Target '{self.target_type}' visible. Sway Power: {self.sway_power}")

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision,
                config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.heading_to_hold is None:
             print("WARN: SwayUntilTargetLost heading_to_hold is None, using current.")
             self.heading_to_hold = sensors.heading
        currently_visible = False
        if self.target_type == 'gate':
             currently_visible = vision.is_gate_visible()
        elif self.target_type == 'pole':
             currently_visible = vision.is_pole_visible()
        else: # either
             currently_visible = vision.is_gate_visible() or vision.is_pole_visible()
        if currently_visible:
            self.target_was_visible = True
            commands = sub.get_heading_commands(sensors, self.heading_to_hold,
                                                surge_power=0.0,
                                                sway_power=self.sway_power,
                                                target_depth=self.target_depth)
            return SubtaskStatus.RUNNING, commands
        else:
            print(f"INFO: SwayUntilTargetLost complete. Target '{self.target_type}' lost.")
            context['initial_heading'] = sensors.heading
            return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)

    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        return f"{self.name}({self.sway_power:+.1f})"


# --- Navigation Subtasks ---
# ... (TurnToHeading, DriveStraight, Stabilize remain the same) ...
class TurnToHeading(Subtask):
    def __init__(self, absolute_degrees: Optional[float] = None, relative_degrees: Optional[float] = None, tolerance_degrees: float = 5.0):
        if absolute_degrees is not None and relative_degrees is not None: raise ValueError("Provide either absolute_degrees or relative_degrees, not both.")
        if absolute_degrees is None and relative_degrees is None: raise ValueError("Must provide either absolute_degrees or relative_degrees.")
        self.absolute_degrees=absolute_degrees; self.relative_degrees=relative_degrees; self.tolerance=tolerance_degrees
        self.target_heading: Optional[float] = None
        self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        if self.absolute_degrees is not None: self.target_heading = self.absolute_degrees % 360
        elif self.relative_degrees is not None:
            initial_heading = context.get('initial_heading', sensors.heading); self.target_heading = (initial_heading + self.relative_degrees) % 360
        else: self.target_heading = None
        self.target_depth = context.get('target_depth', sensors.depth)
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.target_heading is None: print("ERROR: TurnToHeading target could not be determined!"); return SubtaskStatus.FAILED, sub._get_damping_commands(sensors)
        heading_error = angle_diff(self.target_heading, sensors.heading)
        if abs(heading_error) < self.tolerance: return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        commands = sub.get_heading_commands(sensors, self.target_heading, surge_power=0.0, target_depth=self.target_depth)
        return SubtaskStatus.RUNNING, commands
    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        if self.target_heading is not None: return f"{self.name}({self.target_heading:.0f}°)"
        elif self.absolute_degrees is not None: return f"{self.name}(Abs {self.absolute_degrees:.0f}°)"
        elif self.relative_degrees is not None: rel = self.relative_degrees; init = context.get('initial_heading', '?'); calc_target = f" -> {(init + rel) % 360:.0f}" if init != '?' else ""; return f"{self.name}(Rel {rel:.0f}° from {init}°{calc_target})"
        return self.name

class DriveStraight(Subtask):
    def __init__(self, duration: float, surge_power: float = 0.5):
        self.duration=duration; self.surge_power=surge_power; self.timer=0.0
        self.heading_to_hold=None; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.timer = self.duration; self.heading_to_hold = context.get('initial_heading', sensors.heading)
        self.target_depth = context.get('target_depth', sensors.depth)
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.timer <= 0: return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        self.timer -= dt
        if self.heading_to_hold is None: self.heading_to_hold = sensors.heading
        commands = sub.get_heading_commands(sensors, self.heading_to_hold, self.surge_power, target_depth=self.target_depth)
        return SubtaskStatus.RUNNING, commands

class Stabilize(Subtask):
    def __init__(self, duration: float = 2.0, speed_threshold: float = 0.05):
        self.duration=duration; self.speed_threshold=speed_threshold; self.timer=0.0
        self.target_set=False; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.timer=0.0; self.target_set=False; self.target_depth = context.get('target_depth', sensors.depth)
        self.target_set = True
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if not self.target_set: self.on_enter(sub, sensors, vision_data, context)
        self.timer += dt
        speed_xy = math.hypot(sensors.velocity_x, sensors.velocity_y); speed_z = abs(sensors.velocity_z)
        if self.timer > self.duration and speed_xy < self.speed_threshold and speed_z < self.speed_threshold: return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        return SubtaskStatus.RUNNING, sub._get_damping_commands(sensors)

# --- Vision-Based Subtasks ---

class DriveUntilTargetLost(Subtask):
    def __init__(self, surge_power: float = 0.5, target_type: str = 'gate'):
        if target_type not in ['gate', 'pole', 'either']: raise ValueError("target_type must be 'gate', 'pole', or 'either'")
        self.surge_power = surge_power; self.target_type = target_type
        self.heading_to_hold: Optional[float] = None; self.target_was_visible = False; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision: Vision, context: Dict[str, Any]):
        self.heading_to_hold = context.get('initial_heading', sensors.heading)
        self.target_depth = context.get('target_depth', sensors.depth)
        if self.target_type == 'gate': self.target_was_visible = vision.is_gate_visible()
        elif self.target_type == 'pole': self.target_was_visible = vision.is_pole_visible()
        else: self.target_was_visible = vision.is_gate_visible() or vision.is_pole_visible()
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.heading_to_hold is None: self.heading_to_hold = sensors.heading
        currently_visible = False
        if self.target_type == 'gate': currently_visible = vision.is_gate_visible()
        elif self.target_type == 'pole': currently_visible = vision.is_pole_visible()
        else: currently_visible = vision.is_gate_visible() or vision.is_pole_visible()
        if currently_visible:
            self.target_was_visible = True
            commands = sub.get_heading_commands(sensors, self.heading_to_hold, self.surge_power, target_depth=self.target_depth)
            return SubtaskStatus.RUNNING, commands
        else:
            if self.target_was_visible: return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
            else: return SubtaskStatus.FAILED, sub._get_damping_commands(sensors)

class WaitForTargetVisible(Subtask):
    # timeout -> FAILED: aborts the task (mission ends, thrusters go idle, the
    # positively-buoyant sub surfaces). Without it a persistently-suppressed
    # target (e.g. a vision false-classification) means hovering/spinning
    # until the battery dies. Timer resets on (re-)entry.
    def __init__(self, target_type='either', timeout: float = 120.0):
        self.target_type = target_type.lower(); self.target_depth: float = 0.1
        self.timeout = timeout; self.timer = 0.0
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.target_depth = context.get('target_depth', sensors.depth)
        self.timer = 0.0
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        self.timer += dt
        visible = False
        if self.target_type == 'pole': visible = vision_data.is_pole_visible()
        elif self.target_type == 'gate': visible = vision_data.is_gate_visible()
        else: visible = vision_data.is_pole_visible() or vision_data.is_gate_visible()
        if visible: return SubtaskStatus.COMPLETED, sub.get_spin_damping_commands(sensors, self.target_depth)
        if self.timer > self.timeout:
            print(f"ERROR: WaitForTargetVisible('{self.target_type}') timed out after {self.timeout:.0f}s")
            return SubtaskStatus.FAILED, sub._get_damping_commands(sensors)
        heave, roll = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0); return SubtaskStatus.RUNNING, sub._mix_and_normalize_commands(0, 0, 0, heave, roll)
    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        return f"{self.name}({self.target_type} {self.timer:.0f}/{self.timeout:.0f}s)"

class AlignToObjectX(Subtask):
    def __init__(self, target_x_fraction: float, tolerance_px: int = 15, yaw_gain: float = 0.8, yaw_rate_tolerance: float = 0.05):
        self.target_x_fraction=target_x_fraction; self.tolerance_px=tolerance_px; self.yaw_gain=yaw_gain; self.yaw_rate_tolerance=yaw_rate_tolerance
        self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.target_depth = context.get('target_depth', sensors.depth)
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        current_center_x = None
        if vision_data.is_gate_visible(): current_center_x = vision_data.get_gate_center_x()
        elif vision_data.is_pole_visible(): current_center_x = vision_data.get_pole_center_x()
        if current_center_x is None: return SubtaskStatus.RUNNING, sub.get_spin_damping_commands(sensors, self.target_depth)
        cam_w = sensors.camera_image.shape[1]; target_pixel_x = cam_w * self.target_x_fraction
        pixel_error_x = current_center_x - target_pixel_x
        yaw = float(np.clip(
            -(pixel_error_x / (cam_w / 2)) * self.yaw_gain - sensors.imu.gyro_z * sub.YAW_D_GAIN,
            -1.0, 1.0
        ))
        is_centered = abs(pixel_error_x) < self.tolerance_px; is_stable = abs(sensors.imu.gyro_z) < self.yaw_rate_tolerance
        if is_centered and is_stable: context['initial_heading'] = sensors.heading; return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        heave, roll = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0)
        return SubtaskStatus.RUNNING, sub._mix_and_normalize_commands(0.0, 0.0, yaw, heave, roll)
    def on_exit(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]): context['initial_heading'] = sensors.heading

class ApproachAndCenterObject(Subtask):
    def __init__(self, height_threshold_px: int, target_x_fraction: float = 0.5, surge_p_gain: float = 0.1, height_tolerance_px: int = 5, yaw_gain: float = 1.5, align_tolerance_px: int = 15, lost_timeout: float = 2.0):
        self.height_threshold = height_threshold_px; self.target_x_fraction = target_x_fraction
        self.surge_p_gain = surge_p_gain; self.height_tolerance = height_tolerance_px
        self.yaw_gain = yaw_gain; self.align_tolerance_px = align_tolerance_px
        self.lost_timeout = lost_timeout; self.time_since_target_lost = 0.0; self.target_depth: float = 0.1
    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.time_since_target_lost = 0.0; self.target_depth = context.get('target_depth', sensors.depth)
        print(f"INFO: Approach starting. Target Height: {self.height_threshold}px, Target X: {self.target_x_fraction*100:.0f}%")
    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        target_height = vision_data.get_pole_apparent_height(); target_center_x = vision_data.get_pole_center_x(); is_visible = vision_data.is_pole_visible()
        if is_visible and target_center_x is not None:
            self.time_since_target_lost = 0.0
            height_error = self.height_threshold - target_height; surge = height_error * self.surge_p_gain; surge = np.clip(surge, -0.4, 0.5)
            cam_w = sensors.camera_image.shape[1]; target_pixel_x = cam_w * self.target_x_fraction
            pixel_error_x = target_center_x - target_pixel_x
            yaw = float(np.clip(
                -(pixel_error_x / (cam_w / 2)) * self.yaw_gain - sensors.imu.gyro_z * sub.YAW_D_GAIN,
                -1.0, 1.0
            ))
            heave, roll = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0)
            is_at_dist = abs(height_error) < self.height_tolerance; is_aligned = abs(pixel_error_x) < self.align_tolerance_px
            if is_at_dist and is_aligned: print("INFO: Approach complete."); context['initial_heading'] = sensors.heading; return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
            return SubtaskStatus.RUNNING, sub._mix_and_normalize_commands(surge, 0.0, yaw, heave, roll)
        else:
            self.time_since_target_lost += dt
            if self.time_since_target_lost > self.lost_timeout: print(f"ERROR: Approach failed - target lost for > {self.lost_timeout}s"); return SubtaskStatus.FAILED, sub.get_spin_damping_commands(sensors, self.target_depth)
            return SubtaskStatus.RUNNING, sub.get_spin_damping_commands(sensors, self.target_depth)
    def on_exit(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        if not self.time_since_target_lost > 0: context['initial_heading'] = sensors.heading

# --- StyleRollSubtask ---
class StyleRollSubtask(Subtask):
    """
    Roll the sub through `degrees` (e.g. 720 = two full rolls) then level out.

    The accumulated angle is integrated from imu.gyro_x rather than read from
    sensors.roll: the vehicle's roll channel is derived from an euler asin in
    sensor_bridge and FOLDS at +/-90 deg, so it cannot track a full rotation.
    The folded channel is only used for the final leveling PD, where (after a
    multiple of 360 deg) the true angle is near 0 and the fold is harmless.

    Heave is scaled by sign/max(0.3, |cos(roll)|) during the spin so the
    vertical thrusters keep pushing the right way in the world frame while
    inverted. Heading is held on context['initial_heading'] (set by the
    preceding AlignToObjectX), so the sub stays pointed at the gate.

    On timeout the subtask COMPLETES rather than fails — the roll is for
    style points and must never kill the mission.
    """
    def __init__(self, degrees: float = 720.0, roll_power: float = 0.9,
                 settle_deg: float = 3.0, settle_rate: float = 0.1,
                 timeout: float = 120.0):
        self.degrees = degrees
        self.roll_power = roll_power
        self.settle_deg = settle_deg
        self.settle_rate = settle_rate
        self.timeout = timeout
        self.target_depth: float = 0.1
        self.reset()

    def reset(self):
        self.accum = 0.0
        self.timer = 0.0
        self.spinning = True
        self.hold_heading: Optional[float] = None

    def on_enter(self, sub: 'Submarine', sensors: SensorSuite, vision_data: Vision, context: Dict[str, Any]):
        self.reset()
        self.target_depth = context.get('target_depth', sensors.depth)
        self.hold_heading = context.get('initial_heading', sensors.heading)
        print(f"INFO: StyleRoll starting: {self.degrees:.0f} deg at depth {self.target_depth:.2f} m")

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision_data: Vision, config: SimulationConfig, context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if self.hold_heading is None:
            self.on_enter(sub, sensors, vision_data, context)
        self.timer += dt
        self.accum += math.degrees(sensors.imu.gyro_x) * dt

        if self.timer > self.timeout:
            print(f"WARN: StyleRoll timed out at {self.accum:.0f}/{self.degrees:.0f} deg — completing anyway")
            return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)

        yaw = float(np.clip(
            math.radians(angle_diff(self.hold_heading, sensors.heading)) * sub.HOVER_YAW_P_GAIN
            - sensors.imu.gyro_z * sub.YAW_D_GAIN, -1.0, 1.0))

        if self.spinning:
            if abs(self.accum) >= self.degrees - 15.0:
                self.spinning = False
                return SubtaskStatus.RUNNING, sub._get_damping_commands(sensors)
            # World-frame-correct heave while rotating: cos from the integrated
            # angle (the folded sensor channel is wrong past +/-90).
            cos_roll = math.cos(math.radians(self.accum))
            sign = 1.0 if cos_roll >= 0 else -1.0
            heave_scale = sign / max(0.3, abs(cos_roll))
            heave, _ = sub.get_depth_roll_commands(sensors, self.target_depth)
            heave = float(np.clip(heave * heave_scale, -1.0, 1.0))
            return SubtaskStatus.RUNNING, sub._mix_and_normalize_commands(
                0.0, 0.0, yaw, heave, self.roll_power)

        # Leveling: near a multiple of 360 the folded roll channel is valid
        if (abs(sensors.roll) < self.settle_deg
                and abs(sensors.imu.gyro_x) < self.settle_rate):
            print(f"INFO: StyleRoll complete: {self.accum:.0f} deg accumulated, "
                  f"leveled at {sensors.roll:+.1f} deg")
            return SubtaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        heave, roll_cmd = sub.get_depth_roll_commands(sensors, self.target_depth, 0.0)
        return SubtaskStatus.RUNNING, sub._mix_and_normalize_commands(
            0.0, 0.0, yaw, heave, roll_cmd)

    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        phase = "spin" if self.spinning else "level"
        return f"{self.name}({self.accum:+.0f}/{self.degrees:.0f} deg {phase})"
