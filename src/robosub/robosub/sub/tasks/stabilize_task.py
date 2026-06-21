#!/usr/bin/env python3
"""
Brings the submarine to a full stop using velocity damping.

Uses _get_damping_commands() to damp all 6 axes of motion rather than
holding a fixed XY position. This is appropriate for hardware where
ground-truth position is not available. The sub stops approximately
where it is and holds depth and heading.
"""
import math
from typing import Tuple

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig


class StabilizeTask(Task):
    """
    Damps all velocity to zero, holds depth and heading.

    Completes when:
      - at least `duration` seconds have elapsed, AND
      - XY speed is below `speed_threshold`, AND
      - vertical speed is below `speed_threshold`
    """

    def __init__(self,
                 duration: float = 3.0,
                 speed_threshold: float = 0.05,
                 target_depth: float = 1.0):

        self.target_depth      = target_depth
        self.STABILIZE_DURATION = duration
        self.SPEED_THRESHOLD    = speed_threshold

        self.state_timer    = 0.0
        self._current_speed   = 0.0
        self._current_speed_z = 0.0

        super().__init__()
        self.reset()

    def reset(self, search_direction: int = 1):
        super().reset(search_direction)
        self.state_timer      = 0.0
        self._current_speed   = 0.0
        self._current_speed_z = 0.0
        self.context['target_depth'] = self.target_depth

    @property
    def state_name(self) -> str:
        return (f"STABILIZING "
                f"(XY: {self._current_speed:.2f} "
                f"Z: {self._current_speed_z:.2f})")

    def execute(self,
                sub: 'Submarine',
                dt: float,
                sensors: SensorSuite,
                vision_data: Vision,
                config: SimulationConfig
                ) -> Tuple[TaskStatus, ThrusterCommands]:

        # Lock heading on the first tick so we hold it throughout
        if self.state_timer == 0.0:
            sub.target_heading = sensors.heading
        sub.target_depth = self.target_depth

        self.state_timer += dt

        speed_xy = math.hypot(sensors.velocity_x, sensors.velocity_y)
        speed_z  = abs(sensors.velocity_z)
        self._current_speed   = speed_xy
        self._current_speed_z = speed_z

        # get_heading_commands: P+D yaw hold, PID depth to target, sway damping
        commands = sub.get_heading_commands(
            sensors,
            heading=sub.target_heading,
            surge_power=0.0,
            target_depth=self.target_depth,
        )

        depth_error = abs(self.target_depth - sensors.depth)
        if (self.state_timer > self.STABILIZE_DURATION
                and speed_xy    < self.SPEED_THRESHOLD
                and speed_z     < self.SPEED_THRESHOLD
                and depth_error < 0.15):
            return TaskStatus.COMPLETED, commands

        return TaskStatus.RUNNING, commands