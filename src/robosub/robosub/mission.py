#!/usr/bin/env python3
"""
Mission plan. Edit this file to change the mission without touching node or
task code.

ROBOSUB_MISSION selects the plan:
  hold    — station-keeping only (used by motortest.sh default).
  orbit   — LEGACY pre-qual course: gate -> orbit the marker pole -> return
            gate. Use this on the physical pre-qual pool layout.
  (else)  — FULL course (default, matches the desktop RoboSim mission):
            gate (with style roll) -> sweep -> slalom out -> slalom return
            (opposite side) -> return gate -> timed hover -> surface ->
            shutdown. Requires red/white slalom poles on the course.

Set ROBOSUB_STYLE_ROLL=720 to add a barrel roll (style points) in front of the
outbound gate (0 disables). motortest.sh exports this (STYLE_ROLL env,
default 720) now that the prequal bring-up runs closed-loop fused depth
(2026-07-03), matching the sim code path. Sim-verified; the remaining
hardware unknown is physical roll authority vs the sub's righting moment
(not modeled in sim) — the subtask times out to COMPLETED (75 s) so a roll
the sub can't finish never strands the mission. First water run: watch it,
and set STYLE_ROLL=0 if the sub can't get past ~90 deg.
"""
import os

from robosub.sub.tasks.stabilize_task import StabilizeTask
from robosub.sub.tasks.gate_task import GateTask
from robosub.sub.tasks.orbit_turn_task import OrbitTurnTask
from robosub.sub.tasks.shutdown_task import ShutdownTask
from robosub.sub.tasks.slalom_task import SlalomTask
from robosub.sub.tasks.sweep_task import SweepForSlalomTask

# ---------------------------------------------------------------------------
# Depths
# ---------------------------------------------------------------------------
MISSION_DEPTH = 1.5   # meters — running depth for all tasks
SURFACE_DEPTH = 0.1   # meters — come back up at end of run

# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def create_mission():
    mode = os.environ.get('ROBOSUB_MISSION', '').lower()

    # Station-keep / hold mode: a single StabilizeTask that never completes.
    # The sub locks heading on the first tick and damps toward target_roll=0,
    # so it actively rejects yaw + roll disturbances (closed-loop from the
    # IMU). Used by .devcontainer/motortest.sh default.
    if mode == 'hold':
        return [StabilizeTask(duration=1e9, target_depth=MISSION_DEPTH)]

    # Barrel roll before the outbound gate — env-gated (see header)
    style_roll = float(os.environ.get('ROBOSUB_STYLE_ROLL', '0') or '0')

    return_gate = GateTask(target_depth=MISSION_DEPTH)
    return_gate.search_direction = -1   # search opposite direction on return

    # LEGACY pre-qual course: gate -> orbit marker pole -> return gate.
    if mode == 'orbit':
        return [
            StabilizeTask(target_depth=MISSION_DEPTH),
            GateTask(target_depth=MISSION_DEPTH, style_roll_degrees=style_roll),
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            OrbitTurnTask(
                target_depth=MISSION_DEPTH,
                approach_height_px=180,
                orbit_target_fraction=0.5,
                orbit_sway_power=-0.15,
                orbit_yaw_gain=2.5,
                min_orbit_time=8.0,
                orbit_target_width_fraction=0.20,
                final_sway_power=-0.5,
            ),
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            return_gate,
            StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),
            ShutdownTask(),
        ]

    # FULL course (default) — mirrors the desktop RoboSim mission:
    #   gate(+roll) -> stabilize -> sweep -> slalom out -> stabilize ->
    #   slalom return (opposite side) -> return gate -> timed hover ->
    #   surface -> shutdown.
    # StabilizeTask doubles as RoboSim's TimedHoverTask (duration at depth)
    # and SurfaceTask (duration at SURFACE_DEPTH) — it already gates on both
    # elapsed time and depth error.
    slalom_out = SlalomTask(target_depth=MISSION_DEPTH)
    slalom_back = SlalomTask(target_depth=MISSION_DEPTH, reversed=True,
                             partner=slalom_out)

    return [
        # Dive and hold before starting
        StabilizeTask(target_depth=MISSION_DEPTH),

        # Pass through the gate (720-degree style roll in front of it)
        GateTask(target_depth=MISSION_DEPTH, style_roll_degrees=style_roll),
        StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),

        # Sweep until the slalom poles are in view, then run the lane
        SweepForSlalomTask(target_depth=MISSION_DEPTH),
        slalom_out,
        StabilizeTask(duration=3.0, target_depth=MISSION_DEPTH),

        # Return leg: opposite side of the poles, then back through the gate
        slalom_back,
        return_gate,

        # Timed hover, surface, end
        StabilizeTask(duration=10.0, target_depth=MISSION_DEPTH),
        StabilizeTask(duration=2.0, target_depth=SURFACE_DEPTH),
        ShutdownTask(),
    ]
