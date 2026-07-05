#!/usr/bin/env python3
"""
Contains all constants, tuning parameters, and configuration classes for the simulation.
"""
from dataclasses import dataclass
from typing import Tuple

# --- Constants and Configuration ---
# --- MODIFIED: Added GREEN to the import ---
WHITE, BLACK, BLUE, LIGHT_BLUE = (255, 255, 255), (0, 0, 0), (20, 50, 120), (100, 150, 200)
RED, GREEN, YELLOW, GRAY = (255, 0, 0), (50, 200, 50), (255, 255, 0), (128, 128, 128)
ORANGE = (255, 165, 0)
CONTROL_BOX_GRAY = (80, 80, 80)
SHARK_BLUE, SAWFISH_GREEN = (70, 100, 150), (100, 140, 100)
POOL_FLOOR_COLOR, WATER_COLOR = (40, 60, 100), (20, 50, 120)
MAGENTA = (255, 0, 255)
GATE_PICTURE_COLOR = (210, 180, 60)   # task-placard color — distinct from RED/WHITE so vision ignores it

# --- Course vision colors (H: 0-360, S: 0-100, V: 0-100) -------------------
# The 2026 course uses RED gate posts + RED/WHITE slalom poles; the marker
# channel is therefore WHITE. For a green-marker course, point
# MARKER_HSV_RANGE at GREEN_MARKER_HSV_RANGE instead.
RED_HSV_RANGES = [((0, 40, 40), (15, 100, 100)), ((340, 40, 40), (360, 100, 100))]
WHITE_HSV_RANGE = [((0, 0, 70), (360, 25, 100))]
GREEN_MARKER_HSV_RANGE = [((100, 40, 40), (140, 100, 100))]
MARKER_HSV_RANGE = WHITE_HSV_RANGE
# Legacy alias — older code imports this name for the marker channel.
GREEN_HSV_RANGE = MARKER_HSV_RANGE


@dataclass
class SimulationConfig:
    worldWidth: float = 40.0
    worldHeight: float = 15.0
    worldDepth: float = 2.1 # 7ft
    cameraFov: float = 70.0
    submarineWidth: float = 0.46
    submarineLength: float = 0.457

    # --- NEW 3D Physics Properties ---
    subMass: float = 4.0        # Mass (kg) 
    subInertia_Z: float = 0.35  # Rotational Inertia (Yaw)
    subInertia_Y: float = 0.35  # Rotational Inertia (Pitch) — kept but no longer driven
    subInertia_X: float = 0.35  # Rotational Inertia (Roll)
    
    thrusterMaxForce: float = 0.8 # Max force per thruster (N)
    # Yaw moment arm (m): lever from the vertical axis to a corner
    # thruster's line of action (~half the hull diagonal). The physics
    # previously used an implicit 1.0 m arm — ~3x optimistic yaw authority
    # on a 0.46 m vehicle.
    yawMomentArm: float = 0.32
    
    # Drag Coefficients (TUNE THESE)
    surgeDragCoeff: float = 1.5   # Forward/backward 
    swayDragCoeff: float = 8.0    # Side-to-side
    heaveDragCoeff: float = 8.0   # Up/down (Assumed same as sway)
    
    angularDragCoeff_Z: float = 3.0  # Yaw drag
    angularDragCoeff_Y: float = 3.0  # Pitch drag — kept but no longer driven
    angularDragCoeff_X: float = 3.0  # Roll drag
    
    # Buoyancy
    gravity: float = 9.81
    # --- MODIFIED: 0.0039 -> 0.0041 for ~1N positive buoyancy ---
    subVolume: float = 0.0041 # (m^3) e.g., 4.1L
    # ---
    waterDensity: float = 1000.0 # (kg/m^3)
    # ---

# --- Pre-Qualification Course Configuration ---
@dataclass
class PrequalConfig:
    # 1. Gate
    GATE_WIDTH_METERS: float = 2.0      # 6.6 ft
    GATE_DEPTH_METERS: float = 1.0      # 3.3 ft below surface
    GATE_OPENING_HEIGHT: float = 1.5    # Your choice, 1.5m
    GATE_COLOR: Tuple[int, int, int] = (255, 0, 0) # RED
    # Task-1-style trim: short divider hanging from the bar's midpoint, and
    # two placard "pictures" hanging near the posts.
    GATE_DIVIDER_HEIGHT: float = 0.61       # ~2ft, real RoboSub gate divider spec
    GATE_PICTURE_WIDTH: float = 0.3
    GATE_PICTURE_HEIGHT: float = 0.3
    GATE_PICTURE_OFFSET_FRAC: float = 0.55  # fraction of half-width from center
    
    # 2. Marker
    MARKER_DIAMETER_METERS: float = 0.3 # Your choice
    MARKER_COLOR: Tuple[int, int, int] = GREEN # <-- Use constant
    
    # 3. Layout (in meters)
    GATE_X_POS: float = 10.0
    MARKER_X_POS: float = GATE_X_POS + 10.0 # 33 ft beyond
    
    # 4. Starting Position
    START_X_POS: float = 7.0 # 3m behind gate
    START_Z_POS: float = 0.1 # Start on the surface
    
    # 5. Pole Extension
    POLE_ABOVE_SURFACE_METERS: float = 0.6096