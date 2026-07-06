#!/usr/bin/env python3
"""
Contains all constants, tuning parameters, and configuration classes for the simulation.

CALIBRATION (2026-07-06, sysid arc): SimulationConfig() applies two overlays at
construction so the sim tracks the measured vehicle without code edits:
  1. venue — ROBOSUB_VENUE=pool|comp (default: leave the legacy 2.1 m world).
  2. sysid/sim_calibration.yaml `sim:` section (path override:
     ROBOSUB_CALIBRATION) — every fitted parameter lives THERE with provenance,
     nothing calibrated is hardcoded here. Missing file = nominal defaults.
CLI/env always see the post-overlay values. The sim is never made easier by an
overlay — only more honest.
"""
import os
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
    angularDragCoeff_Y: float = 3.0  # Pitch drag (drives the passive pitch DOF)
    angularDragCoeff_X: float = 3.0  # Roll drag

    # --- Passive attitude dynamics (2026-07-06, sysid W5) -------------------
    # The vehicle has NO pitch actuators (verticals are left/right), and it is
    # deliberately ballasted with a SMALL righting moment so the style roll is
    # possible. Pitch is therefore a passive, disturbance-driven DOF that nav
    # must live with — the #1 real-vs-sim gap was surge causing bow-up pitch.
    # ALL FOUR values below are NOMINAL PRIORS awaiting the S2 tilt-release +
    # S5 surge-step fits; the fitted numbers land in sim_calibration.yaml.
    pitchRightingMoment: float = 0.5   # N·m per rad of pitch (restoring)
    rollRightingMoment: float = 0.3    # N·m per rad of roll (restoring; makes
                                       # the style roll cost real torque)
    surgePitchCoupling: float = 0.06   # N·m of bow-up per N of surge thrust
    surgePitchVelCoupling: float = 0.0 # N·m per (m/s)² of surge speed (hydro
                                       # lift term; 0 until S5 says otherwise)

    # Buoyancy
    gravity: float = 9.81
    # --- MODIFIED: 0.0039 -> 0.0041 for ~1N positive buoyancy ---
    subVolume: float = 0.0041 # (m^3) e.g., 4.1L
    # ---
    waterDensity: float = 1000.0 # (kg/m^3)
    # ---

    def __post_init__(self):
        self._apply_venue()
        self._apply_calibration()

    def _apply_venue(self):
        """ROBOSUB_VENUE=pool|comp world geometry. Default: legacy 2.1 m."""
        venue = os.environ.get('ROBOSUB_VENUE', '').lower()
        if venue == 'pool':
            self.worldDepth = 1.52    # the user's pool (5 ft)
        elif venue == 'comp':
            self.worldDepth = 5.8     # competition (to 19 ft)
        if venue:
            print(f"[config] venue '{venue}': worldDepth={self.worldDepth} m",
                  flush=True)

    def _apply_calibration(self):
        """Overlay sysid/sim_calibration.yaml `sim:` values onto this config."""
        path = os.environ.get('ROBOSUB_CALIBRATION',
                              '/home/robosub/robosub_ws/sysid/sim_calibration.yaml')
        try:
            import yaml
            with open(path) as f:
                cal = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return  # nominal defaults — the pre-calibration behavior
        except Exception as e:
            print(f"[config] calibration load FAILED ({path}): {e}", flush=True)
            return
        applied = []
        for key, val in (cal.get('sim') or {}).items():
            if hasattr(self, key):
                setattr(self, key, type(getattr(self, key))(val))
                applied.append(key)
            else:
                print(f"[config] calibration key '{key}' unknown — ignored",
                      flush=True)
        if applied:
            print(f"[config] calibration applied from {path}: {applied}",
                  flush=True)

# --- Pre-Qualification Course Configuration ---
@dataclass
class PrequalConfig:
    # 1. Gate
    # RoboSub 2026 official (team handbook 3.2, fetched 2026-07-06): gate is
    # 120 in x 60 in (3.0 x 1.5 m), buoyant, floats JUST BELOW the surface,
    # moored; pass anywhere from the floor to just below the gate.
    GATE_WIDTH_METERS: float = 3.0      # official 120 in
    GATE_DEPTH_METERS: float = 0.2      # top just below surface (moored float)
    GATE_OPENING_HEIGHT: float = 1.5    # Your choice, 1.5m
    GATE_COLOR: Tuple[int, int, int] = (255, 0, 0) # RED
    # Task-1-style trim: short divider hanging from the bar's midpoint, and
    # two placard "pictures" hanging near the posts.
    GATE_DIVIDER_HEIGHT: float = 0.61       # ~2ft, real RoboSub gate divider spec
    GATE_PICTURE_WIDTH: float = 0.305   # official 12 in placards
    GATE_PICTURE_HEIGHT: float = 0.305
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

    def __post_init__(self):
        # Pool venue (1.52 m water): a floor-standing gate must have its bar
        # shallower or the default mission depth (0.8 m) hits it. 0.5 m is an
        # ASSUMPTION — replace with the real gate's measured bar depth in the
        # user's pool (PROTOCOL A2) via sim_calibration.yaml or here.
        import os
        if os.environ.get('ROBOSUB_VENUE', '').lower() == 'pool':
            self.GATE_DEPTH_METERS = 0.5
            # the user's practice gate is their own build, ~2 m wide (ASSUMED
            # until measured) — comp official 3.0 m stays for the comp venue.
            self.GATE_WIDTH_METERS = 2.0
            print('[config] venue pool: gate bar %.2f m, width %.1f m (ASSUMED — measure the real gate)'
                  % (self.GATE_DEPTH_METERS, self.GATE_WIDTH_METERS), flush=True)