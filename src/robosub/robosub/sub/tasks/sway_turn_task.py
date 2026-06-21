#!/usr/bin/env python3
"""
Sway Turn Task: Navigates around the marker by approaching,
strafing left, and then turning 180 degrees.
"""
import math
from typing import Tuple, List, Dict, Any
import numpy as np

# Absolute imports
from robosub.sub.tasks.task_base import Task, TaskStatus
from robosub.sub.tasks.subtask_base import Subtask, SubtaskStatus
# Import all the building blocks
from robosub.sub.tasks.common_subtasks import (WaitForTargetVisible, AlignToObjectX, 
                                      ApproachAndCenterObject, TurnToHeading, 
                                      SwayStraight, Stabilize) 

from robosub.sub.data_structures import SensorSuite, Vision, ThrusterCommands 
from robosub.sub.config import SimulationConfig 
from robosub.sub.utils import angle_diff

class SwayTurnTask(Task):
    """
    Finds, approaches, sways past, and turns 180 degrees.
    This task assumes the *next* task will be finding the gate.
    """

    # --- Simple subtask to store heading in context ---
    class _StoreHeadingSubtask(Subtask):
        def execute(self, sub: 'Submarine', dt: float, sensors: SensorSuite, vision: Vision, config: SimulationConfig, context: dict) -> Tuple[SubtaskStatus, ThrusterCommands]:
            context['initial_heading'] = sensors.heading
            print(f"INFO: Stored initial heading: {sensors.heading:.1f}")
            return SubtaskStatus.COMPLETED, ThrusterCommands()
    # ---

    def __init__(self, 
                 target_depth: float = 1.0,
                 approach_height_px: int = 80,
                 sway_duration: float = 3.0,
                 sway_power: float = -0.5): # Negative = Left
        
        self.target_depth = target_depth
        
        super().__init__() # Call base constructor

        self.subtasks = [
            # 1. Find the pole
            WaitForTargetVisible(target_type='pole'), 
            
            # 2. Align to it
            AlignToObjectX(
                target_x_fraction=0.5,
                tolerance_px=10,
                yaw_gain=1.5,
                yaw_rate_tolerance=0.05
            ),
            
            # 3. Get close to it
            ApproachAndCenterObject(
                height_threshold_px=approach_height_px,
                surge_p_gain=0.1,
                height_tolerance_px=10,
                yaw_gain=1.5
            ),
            
            # 4. Stop
            Stabilize(duration=1.0),
            
            # 5. Store current heading (should be ~0)
            SwayTurnTask._StoreHeadingSubtask(),
            
            # 6. Strafe left
            SwayStraight(
                duration=sway_duration,
                sway_power=sway_power
            ),
            
            # 7. Stop
            Stabilize(duration=1.0),
            
            # 8. Turn around
            TurnToHeading(
                relative_degrees=180.0, # Turn 180
                tolerance_degrees=10.0
            ),
        ]
        self.reset() # Call our reset

    def reset(self, search_direction: int = 1):
        """Resets the task and adds target_depth to the context."""
        super().reset(search_direction)
        self.context['target_depth'] = self.target_depth

    # No special execute() logic needed. The base class will run
    # subtasks 1-8 and then return COMPLETED. The *next* task
    # in main.py (GateTask) will then be responsible for finding the gate.