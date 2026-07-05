#!/usr/bin/env python3
"""Final mission task: command zero thrust and complete immediately."""
from typing import Tuple

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig


class ShutdownTask(Task):

    def __init__(self):
        super().__init__()

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision,
                config: SimulationConfig) -> Tuple[TaskStatus, ThrusterCommands]:
        print("INFO: ShutdownTask — zero thrust.")
        return TaskStatus.COMPLETED, ThrusterCommands()
