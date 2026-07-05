#!/usr/bin/env python3
"""Base class for reusable sub-actions within a Task.

Lifecycle (driven by Task via tick(), never called directly by subclasses):
    on_enter -> execute ... execute -> on_exit

Every subtask is time-bounded: TIMEOUT (seconds) with an ON_TIMEOUT policy of
'fail' (abort the task — for steps that must succeed, like a compass turn) or
'complete' (safety valve — for steps whose failure must never strand the
mission, like the style roll). A timeout is always loud: it prints, sets
self.timed_out, and the owning Task keeps a persistent marker in its status
string so post-run analysis can tell a clean pass from a bailed one.
"""
from enum import Enum, auto
from typing import Dict, Any, Optional, Tuple

from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig


class SubtaskStatus(Enum):
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()


class Subtask:
    TIMEOUT: Optional[float] = 60.0   # seconds; None = unbounded (avoid)
    ON_TIMEOUT: str = 'fail'          # 'fail' | 'complete'

    def __init__(self):
        self._entered = False
        self._elapsed = 0.0
        self.timed_out = False

    # -- overridable lifecycle ------------------------------------------------

    def on_enter(self, sub: 'Submarine', sensors: SensorSuite,
                 vision: Vision, context: Dict[str, Any]):
        pass

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision, config: SimulationConfig,
                context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        raise NotImplementedError

    def on_exit(self, sub: 'Submarine', sensors: SensorSuite,
                vision: Vision, context: Dict[str, Any]):
        pass

    def reset(self):
        self._entered = False
        self._elapsed = 0.0
        self.timed_out = False

    # -- engine entry point ---------------------------------------------------

    def tick(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
             vision: Vision, config: SimulationConfig,
             context: Dict[str, Any]) -> Tuple[SubtaskStatus, ThrusterCommands]:
        if not self._entered:
            self._entered = True
            self._elapsed = 0.0
            self.timed_out = False
            self.on_enter(sub, sensors, vision, context)

        self._elapsed += dt
        if self.TIMEOUT is not None and self._elapsed > self.TIMEOUT:
            self.timed_out = True
            hold = sub.ctrl.hold(sensors, dt,
                                 context.get('target_depth', sensors.depth))
            if self.ON_TIMEOUT == 'complete':
                print(f"WARN: {self.name} TIMEOUT after {self.TIMEOUT:.0f}s "
                      f"— completing (safety valve)")
                status = SubtaskStatus.COMPLETED
            else:
                print(f"ERROR: {self.name} TIMEOUT after {self.TIMEOUT:.0f}s — failing")
                status = SubtaskStatus.FAILED
            self.on_exit(sub, sensors, vision, context)
            self._entered = False
            return status, hold

        status, commands = self.execute(sub, dt, sensors, vision, config, context)
        if status != SubtaskStatus.RUNNING:
            self.on_exit(sub, sensors, vision, context)
            self._entered = False
        return status, commands

    # -- naming ----------------------------------------------------------------

    @property
    def name(self) -> str:
        return self.__class__.__name__

    def get_dynamic_name(self, context: Dict[str, Any]) -> str:
        return self.name
