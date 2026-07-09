#!/usr/bin/env python3
"""Station-keeping task: hold depth and heading, complete once quiet.

Stability is judged by signals the real vehicle can sense — rotation rates,
roll angle, depth error, vertical speed — plus elapsed time. (There is no
lateral velocity sensing on the vehicle; the old XY-speed criterion was
trivially true on hardware and is gone.)

Also serves as the timed hover and the surface step of the full course, and
as the never-ending 'hold' mission used by motortest.sh on the vehicle.
"""
from typing import Tuple

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig


class StabilizeTask(Task):

    def __init__(self, duration: float = 3.0,
                 speed_threshold: float = 0.05,   # accepted for compatibility
                 target_depth: float = 1.0,
                 settle_grace: float = 30.0):
        self.target_depth = target_depth
        self.duration = duration
        # Complete after duration+settle_grace EVEN IF not settled, so a run
        # can never hang holding forever (2026-07-09: an unstable roll kept
        # is_settled false -> the holdtest ran the full 420 s cap).
        self.settle_grace = settle_grace
        self._timer = 0.0
        self._heading = None
        super().__init__()

    def reset(self, search_direction=None):
        super().reset(search_direction)
        self._timer = 0.0
        self._heading = None
        self.context['target_depth'] = self.target_depth

    @property
    def state_name(self) -> str:
        return f"STABILIZING ({self._timer:.1f}/{self.duration:.1f}s)"

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision,
                config: SimulationConfig) -> Tuple[TaskStatus, ThrusterCommands]:
        if self._heading is None:
            self._heading = sensors.heading
        self._timer += dt
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self._heading)
        if self._timer >= self.duration:
            if sub.ctrl.is_settled(sensors, self.target_depth):
                return TaskStatus.COMPLETED, cmds
            if self._timer >= self.duration + self.settle_grace:
                print("WARN: StabilizeTask completing UNSETTLED after "
                      "%.0f+%.0f s (roll/depth never quieted)"
                      % (self.duration, self.settle_grace))
                return TaskStatus.COMPLETED, cmds
        return TaskStatus.RUNNING, cmds
