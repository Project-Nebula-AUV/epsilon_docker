#!/usr/bin/env python3
"""Mission plans. Edit this file to change the mission; node and task code
stay untouched.

ROBOSUB_MISSION selects the plan:
  hold    — station-keeping forever (motortest.sh default on the vehicle).
  holdtest— WATER SESSION 1 checkout: 60 s depth hold at ROBOSUB_TEST_DEPTH
            (default 1.2 m) -> +90 deg compass turn -> 8 s hold -> surface.
  rolltest— WATER SESSION 1 checkout: settle -> 360 deg style roll -> 8 s
            re-hold -> surface. Uses the pump controller (45 s timeout).
  orbit   — legacy pre-qual: gate -> orbit the marker pole -> return gate.
  gate    — outbound gate only (with style roll if enabled). Test mode.
  slalom  — sweep + slalom out + slalom back. Test mode (start facing the
            lane; the reference heading is captured at start).
  (else)  — FULL course: gate(+roll) -> sweep -> slalom out -> slalom back
            -> return gate -> timed hover -> surface -> shutdown.

ROBOSUB_STYLE_ROLL=720 adds the barrel roll before the outbound gate
(0 disables). The roll subtask times out COMPLETED, so a roll the vehicle
cannot finish never strands the mission.
"""
import os

from robosub.sub.tasks.stabilize_task import StabilizeTask
from robosub.sub.tasks.gate_task import GateTask
from robosub.sub.tasks.orbit_turn_task import OrbitTurnTask
from robosub.sub.tasks.shutdown_task import ShutdownTask
from robosub.sub.tasks.slalom_task import SlalomTask
from robosub.sub.tasks.sweep_task import SweepForSlalomTask
from robosub.sub.tasks.task_base import Task
from robosub.sub.tasks.common_subtasks import TurnToHeading, StyleRollSubtask

MISSION_DEPTH = float(os.environ.get('ROBOSUB_MISSION_DEPTH', '1.5'))  # m — running depth (pool venue: set 0.8)
SURFACE_DEPTH = 0.1    # m — end of run
# Gate crossing depth: centered between the bar (~1.0 m) and the floor
# (~2.1 m) so the hull clears with margin both ways.
GATE_CENTER_DEPTH = float(os.environ.get('ROBOSUB_GATE_DEPTH', '1.55'))  # pool venue: ~1.0 (bar 0.5, floor 1.52)


def create_mission():
    mode = os.environ.get('ROBOSUB_MISSION', '').lower()
    style_roll = float(os.environ.get('ROBOSUB_STYLE_ROLL', '0') or '0')

    if mode == 'hold':
        return [StabilizeTask(duration=1e9, target_depth=MISSION_DEPTH)]

    test_depth = float(os.environ.get('ROBOSUB_TEST_DEPTH', '1.2'))

    if mode == 'holdtest':
        # Water-1-loop checkout: THE gate for everything else. PASS = depth
        # within +-10 cm for the 60 s hold + a clean 90 deg turn.
        return [
            StabilizeTask(duration=60.0, target_depth=test_depth),
            Task([TurnToHeading(relative_degrees=90.0)]),
            StabilizeTask(duration=8.0, target_depth=test_depth),
            StabilizeTask(duration=2.0, target_depth=SURFACE_DEPTH),
            ShutdownTask(),
        ]

    if mode == 'rolltest':
        # Style-roll controller checkout (physics proven by S9): settle,
        # one 360, re-level, re-hold, surface. Roll subtask times out
        # COMPLETED so a failure cannot strand the run.
        return [
            StabilizeTask(duration=20.0, target_depth=test_depth),
            Task([StyleRollSubtask(degrees=360.0, timeout=45.0,
                                   target_depth=test_depth)]),
            StabilizeTask(duration=8.0, target_depth=test_depth),
            StabilizeTask(duration=2.0, target_depth=SURFACE_DEPTH),
            ShutdownTask(),
        ]

    gate = GateTask(target_depth=GATE_CENTER_DEPTH, side='right',
                    style_roll_degrees=style_roll)
    return_gate = GateTask(target_depth=GATE_CENTER_DEPTH, side='right',
                           reverse=True, search_direction=-1)

    if mode == 'orbit':
        return [
            StabilizeTask(target_depth=MISSION_DEPTH),
            gate,
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            OrbitTurnTask(target_depth=MISSION_DEPTH,
                          approach_height_px=180,
                          orbit_target_width_fraction=0.20,
                          orbit_sway_power=-0.15,
                          min_orbit_time=8.0,
                          final_sway_power=-0.5),
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            return_gate,
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            ShutdownTask(),
        ]

    if mode == 'gate':
        return [
            StabilizeTask(target_depth=MISSION_DEPTH),
            gate,
            StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),
            ShutdownTask(),
        ]

    if mode == 'slalom':
        return [
            StabilizeTask(target_depth=MISSION_DEPTH),
            SweepForSlalomTask(target_depth=MISSION_DEPTH),
            SlalomTask(target_depth=MISSION_DEPTH),
            StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),
            SlalomTask(target_depth=MISSION_DEPTH, reversed=True),
            StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),
            ShutdownTask(),
        ]

    # FULL course (default)
    return [
        StabilizeTask(target_depth=MISSION_DEPTH),
        gate,
        StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),
        SweepForSlalomTask(target_depth=MISSION_DEPTH),
        SlalomTask(target_depth=MISSION_DEPTH),
        StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),
        SlalomTask(target_depth=MISSION_DEPTH, reversed=True),
        return_gate,
        StabilizeTask(duration=10.0, target_depth=MISSION_DEPTH),
        StabilizeTask(duration=2.0, target_depth=SURFACE_DEPTH),
        ShutdownTask(),
    ]
