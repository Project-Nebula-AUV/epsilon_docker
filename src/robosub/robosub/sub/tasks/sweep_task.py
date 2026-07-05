#!/usr/bin/env python3
"""Sweep task: bounded yaw sweep at depth until a slalom gatelet is in view.

Runs between the gate and the slalom so SlalomTask starts with poles visible.
The sweep is centered on the COURSE AXIS (mission reference heading), not on
whatever heading the previous task ended at, and is bounded so the gate
behind the vehicle never enters the camera. Timeout completes — the slalom's
own search is the fallback.
"""
from typing import Optional, Tuple

import numpy as np

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig
from robosub.sub.utils import angle_diff


class SweepForSlalomTask(Task):

    def __init__(self, target_depth: float = 1.5,
                 sweep_rate: float = 0.25,
                 max_sweep_degrees: float = 50.0,
                 confirm_ticks: int = 8,
                 timeout: float = 40.0):
        self.target_depth = target_depth
        self.SWEEP_RATE = sweep_rate          # rad/s commanded scan rate
        self.MAX_SWEEP_DEGREES = max_sweep_degrees
        self.CONFIRM_TICKS = confirm_ticks
        self.TIMEOUT = timeout
        self._center: Optional[float] = None
        self._direction = 1
        self._seen = 0
        self._elapsed = 0.0
        super().__init__()

    def reset(self, search_direction=None):
        super().reset(search_direction)
        self._center = None
        self._direction = 1
        self._seen = 0
        self._elapsed = 0.0
        self.context['target_depth'] = self.target_depth

    @property
    def state_name(self) -> str:
        return f"SWEEPING (seen {self._seen}/{self.CONFIRM_TICKS})"

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision, config: SimulationConfig
                ) -> Tuple[TaskStatus, ThrusterCommands]:
        if self._center is None:
            if sub.shared.reference_heading is None:
                # No gate leg ran before us (slalom test mode): the heading
                # at sweep start IS the course axis — lock it for the slalom.
                sub.shared.reference_heading = sensors.heading
                print(f"INFO: course reference locked at sweep start "
                      f"({sensors.heading:.1f} deg)")
            self._center = sub.shared.reference_heading
        self._elapsed += dt

        if vision.get_slalom_gatelet() is not None:
            self._seen += 1
            cmds = sub.ctrl.hold(sensors, dt, self.target_depth)
            if self._seen >= self.CONFIRM_TICKS:
                print("INFO: Sweep found slalom gatelet")
                return TaskStatus.COMPLETED, cmds
            return TaskStatus.RUNNING, cmds
        self._seen = 0

        if self._elapsed > self.TIMEOUT:
            print("WARN: Sweep TIMEOUT without a gatelet — completing (valve)")
            return TaskStatus.COMPLETED, sub.ctrl.hold(sensors, dt,
                                                       self.target_depth)

        rel = angle_diff(sensors.heading, self._center)
        if rel > self.MAX_SWEEP_DEGREES:
            self._direction = -1
        elif rel < -self.MAX_SWEEP_DEGREES:
            self._direction = 1
        return TaskStatus.RUNNING, sub.ctrl.scan(
            sensors, dt, self.target_depth, self.SWEEP_RATE * self._direction)
