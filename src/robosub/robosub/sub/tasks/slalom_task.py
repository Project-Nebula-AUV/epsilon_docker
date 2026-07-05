#!/usr/bin/env python3
"""Slalom task: run the lane of red/white pole gatelets.

State machine per gatelet: SEARCH (cruise the course axis, scan-free — the
lane is dead ahead by construction) -> CENTER (sway the red/white gap middle
onto the frame center, compass square on the axis) -> APPROACH (surge with
sway trim on the gap) -> CLEAR (timed blind surge once the gatelet leaves the
near field) -> SEARCH ...

Navigation references come from MissionShared, not from whatever heading the
previous task ended at: the outbound leg runs on the mission reference
heading (gate normal == lane axis on this course) and RECORDS its own axis +
chosen side; the reversed leg runs the recorded axis + 180 and the opposite
side. Completion reports the honest pass count; finishing with zero passes is
a FAILURE unless the safety timeout forced completion (which is loud).
"""
import math
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np

from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands
from robosub.sub.config import SimulationConfig
from robosub.sub.utils import angle_diff


class S(Enum):
    DIVE = auto()
    SEARCH = auto()
    CENTER = auto()
    APPROACH = auto()
    CLEAR = auto()
    FINISH = auto()


class SlalomTask(Task):

    def __init__(self, target_depth: float = 1.5,
                 reversed: bool = False,
                 surge_power: float = 0.4,
                 clear_duration: float = 2.0,
                 lane_done_secs: float = 9.0,
                 task_timeout: float = 240.0,
                 **_legacy):
        self.target_depth = target_depth
        self.rev = reversed
        self.SURGE = surge_power
        self.CLEAR_SECS = clear_duration
        self.LANE_DONE_SECS = lane_done_secs    # SEARCH dry spell => lane done
        # Cruise only this long into a SEARCH dry spell — just enough to
        # bridge the inter-gatelet spacing (~4 m at ~0.65 m/s), then hold
        # station for the rest of the window. Bounds the overshoot past the
        # LAST gatelet so the lane never spills into the next course element
        # (confirmed: 9 s of full-surge SEARCH carried the return leg ~7 m,
        # straight through the gate, before GateTask had control).
        self.CRUISE_ON_DRY_S = 4.0
        self.TASK_TIMEOUT = task_timeout
        self.CENTER_TOL_FRAC = 0.05             # of image width
        self.LOST_TO_CLEAR_S = 0.6
        super().__init__()

    def reset(self, search_direction=None):
        super().reset(search_direction)
        self.state = S.DIVE
        self.axis: Optional[float] = None
        self.side: Optional[str] = None
        self.passes = 0
        self.t_state = 0.0
        self.t_task = 0.0
        self.t_no_gatelet = 0.0
        self.tracked_h = 0.0
        self.context['target_depth'] = self.target_depth

    @property
    def state_name(self) -> str:
        return (f"{self.state.name} pass={self.passes} "
                f"side={self.side or '?'}{' REV' if self.rev else ''}")

    # -- helpers ------------------------------------------------------------

    def _enter(self, state):
        self.state = state
        self.t_state = 0.0
        self.t_no_gatelet = 0.0

    def _gap_center_err(self, gatelet, cam_w) -> float:
        red, white = gatelet
        mid = (red['center_x'] + white['center_x']) / 2.0
        return (mid - cam_w / 2.0) / (cam_w / 2.0)

    # -- main ---------------------------------------------------------------

    def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite,
                vision: Vision, config: SimulationConfig
                ) -> Tuple[TaskStatus, ThrusterCommands]:
        if self.axis is None:
            base = (sub.shared.slalom_axis if self.rev
                    else sub.shared.reference_heading)
            if base is None:
                base = sensors.heading
            self.axis = (base + 180.0) % 360.0 if self.rev else base
            if self.rev and sub.shared.slalom_side:
                self.side = ('right' if sub.shared.slalom_side == 'left'
                             else 'left')
            print(f"INFO: Slalom start rev={self.rev} axis={self.axis:.0f} "
                  f"side={self.side}")

        self.t_task += dt
        self.t_state += dt
        if self.t_task > self.TASK_TIMEOUT:
            print(f"WARN: Slalom TIMEOUT after {self.passes} passes "
                  f"— completing (valve)")
            self._record(sub)
            return TaskStatus.COMPLETED, sub.ctrl.hold(sensors, dt,
                                                       self.target_depth)

        cam_w = sensors.camera_image.shape[1]
        gatelet = vision.get_slalom_gatelet(self.side)

        if self.state == S.DIVE:
            cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self.axis)
            if (abs(self.target_depth - sensors.depth) < 0.15
                    and abs(angle_diff(self.axis, sensors.heading)) < 6.0):
                self._enter(S.SEARCH)
            return TaskStatus.RUNNING, cmds

        if self.state == S.SEARCH:
            if gatelet:
                if self.side is None:
                    red = gatelet[0]
                    self.side = ('left' if red['center_x'] > cam_w / 2
                                 else 'right')
                    print(f"INFO: Slalom side chosen: {self.side}")
                    gatelet = vision.get_slalom_gatelet(self.side) or gatelet
                sub.ctrl.reset_pixel_servo()
                self._enter(S.CENTER)
                return TaskStatus.RUNNING, sub.ctrl.hold(
                    sensors, dt, self.target_depth, self.axis)
            self.t_no_gatelet += dt
            if self.t_no_gatelet > self.LANE_DONE_SECS:
                self._enter(S.FINISH)
            surge = (self.SURGE if self.t_no_gatelet < self.CRUISE_ON_DRY_S
                     else 0.0)
            return TaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth, self.axis, surge=surge)

        if self.state == S.CENTER:
            if not gatelet:
                self.t_no_gatelet += dt
                if self.t_no_gatelet > 2.0:
                    self._enter(S.SEARCH)
                return TaskStatus.RUNNING, sub.ctrl.hold(
                    sensors, dt, self.target_depth, self.axis)
            self.t_no_gatelet = 0.0
            err = self._gap_center_err(gatelet, cam_w)
            cmds = sub.ctrl.track_pixel_sway(sensors, dt, self.target_depth,
                                             err, self.axis)
            if (abs(err) < self.CENTER_TOL_FRAC
                    and abs(sub.ctrl.pixel_rate()) < 0.05
                    and abs(angle_diff(self.axis, sensors.heading)) < 4.0):
                self._enter(S.APPROACH)
            return TaskStatus.RUNNING, cmds

        if self.state == S.APPROACH:
            if gatelet:
                self.t_no_gatelet = 0.0
                # Passing a gatelet rarely blanks vision: by the time this
                # triplet leaves the FOV the next one is already visible and
                # tracking hands over seamlessly. The handover IS the pass —
                # visible as the tracked red's apparent height collapsing to
                # a much smaller (farther) blob in a single tick.
                red_h = gatelet[0]['height']
                if self.tracked_h > 0 and red_h < 0.55 * self.tracked_h:
                    self.passes += 1
                    print(f"INFO: Slalom pass {self.passes} complete "
                          f"(handover, rev={self.rev})")
                self.tracked_h = red_h
                err = self._gap_center_err(gatelet, cam_w)
                return TaskStatus.RUNNING, sub.ctrl.track_pixel_sway(
                    sensors, dt, self.target_depth, err, self.axis,
                    surge=self.SURGE)
            self.t_no_gatelet += dt
            if self.t_no_gatelet > self.LOST_TO_CLEAR_S:
                self._enter(S.CLEAR)
            return TaskStatus.RUNNING, sub.ctrl.hold(
                sensors, dt, self.target_depth, self.axis, surge=self.SURGE)

        if self.state == S.CLEAR:
            cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self.axis,
                                 surge=self.SURGE)
            if self.t_state > self.CLEAR_SECS:
                self.passes += 1
                self.tracked_h = 0.0
                print(f"INFO: Slalom pass {self.passes} complete "
                      f"(clear, rev={self.rev})")
                # The reversed leg knows how long the lane is (the outbound
                # leg counted it): finish the moment the last gatelet is
                # cleared instead of drifting through a dry-spell window —
                # the next task (return gate) needs the standoff distance.
                expected = sub.shared.slalom_passes_out
                if self.rev and expected > 0 and self.passes >= expected:
                    self._enter(S.FINISH)
                else:
                    self._enter(S.SEARCH)
            return TaskStatus.RUNNING, cmds

        # FINISH: square up on the axis, then report honestly.
        cmds = sub.ctrl.hold(sensors, dt, self.target_depth, self.axis)
        if abs(angle_diff(self.axis, sensors.heading)) < 6.0:
            self._record(sub)
            if self.passes == 0:
                print("ERROR: Slalom finished with ZERO passes")
                return TaskStatus.FAILED, cmds
            print(f"INFO: Slalom complete: {self.passes} passes "
                  f"(rev={self.rev})")
            return TaskStatus.COMPLETED, cmds
        return TaskStatus.RUNNING, cmds

    def _record(self, sub: 'Submarine'):
        """Publish this leg's results to the mission-shared references."""
        if self.rev:
            sub.shared.slalom_passes_back = self.passes
        else:
            sub.shared.slalom_axis = self.axis
            sub.shared.slalom_side = self.side
            sub.shared.slalom_passes_out = self.passes
