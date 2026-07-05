#!/usr/bin/env python3
"""Orbit course task (legacy pre-qual): find the marker pole, approach it,
orbit until the return gate appears to its right, then sway clear of it.
Kept working because ROBOSUB_MISSION=orbit is the physical pre-qual path.
"""
from robosub.sub.tasks.task_base import Task
from robosub.sub.tasks.common_subtasks import (
    AcquireTarget, AlignToObjectX, ApproachAndCenterObject, Stabilize,
    DynamicOrbitPole, SwayUntilTargetLost)


class OrbitTurnTask(Task):

    def __init__(self, target_depth: float = 1.5,
                 approach_height_px: int = 120,
                 orbit_target_fraction: float = 0.5,
                 orbit_sway_power: float = -0.15,
                 orbit_yaw_gain: float = 2.5,          # legacy arg, unused
                 orbit_target_width_fraction: float = 0.06,
                 orbit_width_p_gain: float = 0.03,
                 orbit_width_i_gain: float = 0.01,
                 orbit_width_d_gain: float = 0.05,
                 orbit_width_i_clamp: float = 0.3,
                 min_orbit_time: float = 5.0,
                 final_sway_power: float = -0.3,
                 search_direction: int = 1,
                 **_legacy):
        self.target_depth = target_depth
        subtasks = [
            AcquireTarget(target_type='pole'),
            AlignToObjectX(target_x_fraction=0.5),
            ApproachAndCenterObject(height_threshold_px=approach_height_px),
            Stabilize(duration=1.0),
            DynamicOrbitPole(
                target_x_fraction=orbit_target_fraction,
                sway_power=orbit_sway_power,
                target_pole_width_fraction=orbit_target_width_fraction,
                orbit_width_p_gain=orbit_width_p_gain,
                orbit_width_i_gain=orbit_width_i_gain,
                orbit_width_d_gain=orbit_width_d_gain,
                orbit_width_i_clamp=orbit_width_i_clamp,
                min_orbit_time=min_orbit_time),
            Stabilize(duration=1.0),
            SwayUntilTargetLost(sway_power=final_sway_power,
                                target_type='pole'),
            Stabilize(duration=1.0),
        ]
        super().__init__(subtasks=subtasks, search_direction=search_direction)

    def reset(self, search_direction=None):
        super().reset(search_direction)
        self.context['target_depth'] = self.target_depth
