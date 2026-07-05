#!/usr/bin/env python3
"""
Mission plan for the pre-qualification course.

Edit this file to change the mission without touching node or task code.
"""
from robosub.sub.tasks.stabilize_task import StabilizeTask
from robosub.sub.tasks.gate_task import GateTask
from robosub.sub.tasks.orbit_turn_task import OrbitTurnTask
from robosub.sub.tasks.shutdown_task import ShutdownTask

# ---------------------------------------------------------------------------
# Depths
# ---------------------------------------------------------------------------
MISSION_DEPTH = 1.5   # meters — running depth for all tasks
SURFACE_DEPTH = 0.1   # meters — come back up at end of run

# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def create_mission():
    return_gate = GateTask(target_depth=MISSION_DEPTH)
    return_gate.search_direction = -1   # search opposite direction on return

    return [
        # Dive and hold before starting
        StabilizeTask(target_depth=MISSION_DEPTH),

        # Pass through the gate
        GateTask(target_depth=MISSION_DEPTH),
        StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),

        # Orbit the green pole until the gate reappears, then clear it
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

        # Return through the gate
        return_gate,
        StabilizeTask(duration=2.0, target_depth=MISSION_DEPTH),

        ShutdownTask(),
    ]
