#!/usr/bin/env python3
"""
Sweep task: yaw back and forth at depth until a slalom gatelet (red+white
pole pair) is in view, then stop turned toward it. Ported from RoboSim.

Runs between the gate and the slalom so SlalomTask starts with poles visible
and latches its course axis pointed at the first gatelet. Bounded sweep keeps
the gate behind the sub out of the camera. Timeout COMPLETES (never fails):
the slalom's own search logic is the fallback.
"""
import math
from typing import Optional, Tuple

import numpy as np

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig
from robosub.sub.utils import angle_diff


class SweepForSlalomTask(Task):

    def __init__(self, target_depth: float = 1.0,
                 sweep_command: float = 0.18,
                 max_sweep_degrees: float = 50.0,
                 confirm_ticks: int = 8,
                 timeout: float = 30.0):
        self.target_depth = target_depth
        self.SWEEP_COMMAND = sweep_command
        self.MAX_SWEEP_DEGREES = max_sweep_degrees
        self.CONFIRM_TICKS = confirm_ticks
        self.TIMEOUT = timeout
        super().__init__()
        self.reset()

    def reset(self, search_direction: int = 1):
        super().reset(search_direction)
        self.center_heading: Optional[float] = None
        self.direction = 1
        self.seen_ticks = 0
        self.elapsed = 0.0
        self.context['target_depth'] = self.target_depth

    @property
    def state_name(self) -> str:
        return f"SWEEPING (seen {self.seen_ticks}/{self.CONFIRM_TICKS})"

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision_data: Vision, config: SimulationConfig
                ) -> Tuple[TaskStatus, ThrusterCommands]:
        if self.center_heading is None:
            self.center_heading = sensors.heading
        self.elapsed += dt

        if vision_data.get_slalom_gatelet() is not None:
            self.seen_ticks += 1
            if self.seen_ticks >= self.CONFIRM_TICKS:
                print("INFO: Sweep found slalom gatelet — done")
                return TaskStatus.COMPLETED, sub._get_damping_commands(sensors)
        else:
            self.seen_ticks = 0

        if self.elapsed > self.TIMEOUT:
            print("WARN: Sweep timed out without a gatelet — completing anyway")
            return TaskStatus.COMPLETED, sub._get_damping_commands(sensors)

        rel = angle_diff(sensors.heading, self.center_heading)
        if rel > self.MAX_SWEEP_DEGREES:
            self.direction = -1
        elif rel < -self.MAX_SWEEP_DEGREES:
            self.direction = 1

        if self.seen_ticks > 0:
            # brake the turn while confirming the sighting
            yaw = float(np.clip(-sensors.imu.gyro_z * sub.YAW_D_GAIN, -1.0, 1.0))
        else:
            yaw = self.SWEEP_COMMAND * self.direction

        heave, roll = sub.get_depth_roll_commands(sensors, self.target_depth)
        return TaskStatus.RUNNING, sub._mix_and_normalize_commands(0.0, 0.0, yaw, heave, roll)
