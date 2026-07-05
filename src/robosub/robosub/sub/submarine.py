#!/usr/bin/env python3
"""The Submarine: mission sequencing + shared services for tasks.

Owns exactly three things:
  * the mission plan and its progression (same-tick task handover — no
    zero-thrust ticks between tasks);
  * the MotionController (all control law state, one instance);
  * the MissionShared navigation references + the Vision pipeline.

All control math lives in control.py; all behavior lives in the tasks.
No pygame, no ROS — pure Python. Receives a SensorSuite each tick, returns
ThrusterCommands.
"""
from typing import List, Optional, Tuple

from robosub.sub.control import MotionController, load_params
from robosub.sub.data_structures import ThrusterCommands, SensorSuite, Vision
from robosub.sub.config import SimulationConfig
from robosub.sub.tasks.task_base import Task, TaskStatus, MissionShared


class Submarine:

    def __init__(self, mission_plan: List[Task]):
        params = load_params()
        self.ctrl = MotionController(params)
        self.mission_plan = mission_plan
        self.config = SimulationConfig()
        self.shared = MissionShared()

        self._latest_sensors: Optional[SensorSuite] = None
        self.vision = Vision(
            image_provider=lambda: (self._latest_sensors.camera_image
                                    if self._latest_sensors else None),
            min_pole_pixels=int(params['min_pixels_for_detection']),
            min_gate_pixels=int(params['min_gate_pixels']),
        )
        self.reset()

    def reset(self):
        self.current_task_index = 0
        self.mission_failed = False
        self.shared = MissionShared()
        self.ctrl.reset()
        self._latest_sensors = None
        for task in self.mission_plan:
            task.reset()

    def update(self, dt: float,
               sensors: SensorSuite) -> Tuple[ThrusterCommands, Vision]:
        self._latest_sensors = sensors
        self.vision.update()

        # Same-tick task handover: a completing task immediately yields the
        # tick to the next one (bounded), so depth/heading hold is never
        # dropped across a transition.
        for _ in range(3):
            if self.current_task_index >= len(self.mission_plan):
                return ThrusterCommands(), self.vision
            task = self.mission_plan[self.current_task_index]
            status, commands = task.execute(self, dt, sensors, self.vision,
                                            self.config)
            if status == TaskStatus.RUNNING:
                return commands, self.vision
            if status == TaskStatus.FAILED:
                print(f"ERROR: task {task.__class__.__name__} FAILED — "
                      f"mission over")
                self.mission_failed = True
                self.current_task_index = len(self.mission_plan)
                return self.ctrl.hold(sensors, dt, sensors.depth), self.vision
            print(f"INFO: task {task.__class__.__name__} completed")
            self.current_task_index += 1
        return commands, self.vision

    # -- status ---------------------------------------------------------------

    def get_current_task_name(self) -> str:
        if self.current_task_index < len(self.mission_plan):
            return self.mission_plan[self.current_task_index].__class__.__name__
        return "MISSION_FAILED" if self.mission_failed else "MISSION_COMPLETE"

    def get_current_state_name(self) -> str:
        if self.current_task_index < len(self.mission_plan):
            return self.mission_plan[self.current_task_index].state_name
        return ""
