#!/usr/bin/env python3
"""Mission plans. Edit this file to change the mission; node and task code
stay untouched.

ROBOSUB_MISSION selects the plan:
  hold    — station-keeping forever (motortest.sh default on the vehicle).
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
