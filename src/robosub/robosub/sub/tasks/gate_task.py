#!/usr/bin/env python3
"""Gate task: pass through the committed half of the two-post gate, square
and centered, with an optional style roll performed in front of it.

Geometry: the compass holds the course axis (the mission-wide reference
heading, captured on the first gate approach); vision only ever moves the
vehicle laterally onto the half-opening's center. The return gate runs the
reciprocal axis (reverse=True) and may search in the opposite direction.
"""
from robosub.sub.tasks.task_base import Task
from robosub.sub.tasks.common_subtasks import (
    CaptureReference, DiveToDepth, AcquireTarget, TurnToHeading,
    CenterOnGateHalf, Stabilize, StyleRollSubtask, DriveThroughGate)


class GateTask(Task):

    def __init__(self, target_depth: float = 1.55,
                 side: str = 'right',
                 style_roll_degrees: float = 0.0,
                 style_roll_timeout: float = 90.0,
                 surge_power: float = 0.55,
                 reverse: bool = False,
                 search_direction: int = 1):
        self.target_depth = target_depth
        self.chosen_side = side

        subtasks = [
            CaptureReference(reverse=reverse),
            DiveToDepth(),
            AcquireTarget(target_type='gate'),
            TurnToHeading(use_axis=True),
            # Kill residual velocity from the previous task before pixel
            # work — the sway servo assumes it starts near rest.
            Stabilize(duration=1.0),
            CenterOnGateHalf(side=side),
            Stabilize(duration=1.5),
        ]
        if style_roll_degrees > 0:
            # The roll is flown in 360-degree segments with a re-center
            # between them. Residual lateral velocity at spin entry is
            # unobservable (no lateral velocity sensing) and nothing damps it
            # mid-roll, so drift grows with spin TIME — halving the segment
            # halves the worst-case excursion, and the between-segment
            # re-center (tight rate_tol) restarts the next segment from rest
            # on the aim axis. Each segment still times out COMPLETED.
            remaining = style_roll_degrees
            while remaining > 0:
                seg = min(360.0, remaining)
                remaining -= seg
                subtasks += [
                    # Tighter than default (roll wants a quiet start) but
                    # achievable under the measured compass noise — the old
                    # 0.02/24 was impossible and always burned the timeout.
                    CenterOnGateHalf(side=side, rate_tol=0.06,
                                     hold_ticks=12),
                    StyleRollSubtask(degrees=seg,
                                     timeout=style_roll_timeout / 2),
                    Stabilize(duration=1.5),
                    TurnToHeading(use_axis=True),
                ]
            subtasks += [
                CenterOnGateHalf(side=side),
                Stabilize(duration=1.5),
            ]
        subtasks += [
            DriveThroughGate(side=side, surge_power=surge_power),
        ]
        super().__init__(subtasks=subtasks, search_direction=search_direction)

    def reset(self, search_direction=None):
        super().reset(search_direction)
        self.context['target_depth'] = self.target_depth
