#!/usr/bin/env python3
"""Mission task engine.

A Task is an ordered list of Subtasks (or a subclass overriding execute()
with its own state machine). The engine guarantees:

  * no zero-thrust ticks: when a subtask completes, the next one runs in the
    SAME tick (bounded chain), so control — including depth hold — is never
    dropped across a transition;
  * instance-owned state: nothing mutable lives on the class;
  * search_direction given at construction survives reset() (Submarine resets
    every task at mission start);
  * subtask timeouts are counted and surface persistently in state_name as
    'T!n' so a run that bailed through safety valves is distinguishable from
    a clean one.

Cross-task navigation references (course axis, committed gate side, slalom
side) live on MissionShared, owned by Submarine — typed, in one place,
instead of stringly-typed context keys leaking between tasks.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple, List, Dict, Any, Optional

from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig
from robosub.sub.tasks.subtask_base import Subtask, SubtaskStatus


class TaskStatus(Enum):
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()


@dataclass
class MissionShared:
    """Navigation references shared across tasks for one mission run."""
    reference_heading: Optional[float] = None  # course axis = gate normal (deg)
    committed_side: Optional[str] = None       # gate half committed to
    slalom_side: Optional[str] = None          # side of red poles, outbound
    slalom_axis: Optional[float] = None        # outbound slalom heading (deg)
    slalom_passes_out: int = 0
    slalom_passes_back: int = 0


class Task:
    CHAIN_LIMIT = 4   # max subtask advances within one tick

    def __init__(self, subtasks: Optional[List[Subtask]] = None,
                 search_direction: int = 1):
        self.subtasks: List[Subtask] = list(subtasks or [])
        self._ctor_search_direction = search_direction
        self.current_subtask_index = 0
        self.context: Dict[str, Any] = {}
        self.search_direction = search_direction
        self._timeout_count = 0
        self.reset()

    def reset(self, search_direction: Optional[int] = None):
        self.current_subtask_index = 0
        self.search_direction = (search_direction if search_direction is not None
                                 else self._ctor_search_direction)
        self.context = {
            'target_depth': getattr(self, 'target_depth', 0.1),
            'search_direction': self.search_direction,
        }
        self._timeout_count = 0
        for st in self.subtasks:
            st.reset()

    @property
    def state_name(self) -> str:
        suffix = f" T!{self._timeout_count}" if self._timeout_count else ""
        if self.subtasks and 0 <= self.current_subtask_index < len(self.subtasks):
            st = self.subtasks[self.current_subtask_index]
            return (f"{self.__class__.__name__}[{self.current_subtask_index}]"
                    f":{st.get_dynamic_name(self.context)}{suffix}")
        return self.__class__.__name__ + suffix

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision,
                config: SimulationConfig) -> Tuple[TaskStatus, ThrusterCommands]:
        commands = None
        for _ in range(self.CHAIN_LIMIT):
            if self.current_subtask_index >= len(self.subtasks):
                return TaskStatus.COMPLETED, (
                    commands if commands is not None else
                    sub.ctrl.hold(sensors, dt,
                                  self.context.get('target_depth', sensors.depth)))
            st = self.subtasks[self.current_subtask_index]
            status, commands = st.tick(sub, dt, sensors, vision, config,
                                       self.context)
            if st.timed_out:
                self._timeout_count += 1
            if status == SubtaskStatus.RUNNING:
                return TaskStatus.RUNNING, commands
            if status == SubtaskStatus.FAILED:
                print(f"ERROR: subtask {st.name} FAILED in "
                      f"{self.__class__.__name__}")
                return TaskStatus.FAILED, commands
            self.current_subtask_index += 1
        return TaskStatus.RUNNING, commands
