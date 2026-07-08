#!/usr/bin/env python3
"""
Contains the main SubmarineSimulator class.
This class handles Pygame, rendering, physics, and the main game loop.
"""
import math
import os
import random
import time
from typing import Tuple, Optional

import pygame
import numpy as np

from robosub.sub.config import *
from robosub.sub.world import SubmarinePhysicsState
from robosub.sub.world import PrequalGate, PrequalMarker, SlalomPole
# --- Import Vision ---
from robosub.sub.data_structures import ThrusterCommands, MPU6050Readings, SensorSuite, Vision
# ---
from robosub.sub.submarine import Submarine


class SubmarineSimulator:
    def __init__(self, submarine_ai: Optional[Submarine] = None, width=1200, height=800):
        pygame.init()
        self.width, self.height = width, height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Autonomous Submarine (AUV) Simulator")
        self.clock = pygame.time.Clock()
        self.config = SimulationConfig()
        self.prequal_config = PrequalConfig()
        self.scaleX = width * 0.7 / self.config.worldWidth
        self.scaleY = height * 0.8 / self.config.worldHeight
        self.font = pygame.font.Font(None, 36)
        self.smallFont = pygame.font.Font(None, 24)
        # HW MISMATCH (2026-07-06): real camera now captures 640x320 @ HFOV 74 deg
        # (see sim_calibration.yaml cameraFov). This surface is still 320x240 --
        # the FOV angle matches so fraction-based vision logic is unaffected, but
        # any PIXEL-count threshold (min blob area, px tolerances) sees 2x scale
        # vs hardware. Bump to (640, 320) as part of the W6 vision refit.
        # W6 2026-07-07: 320x160 — the HARDWARE camera publishes 640x320
        # (2:1, HFOV 74 -> VFOV 41.3 deg). The old 320x240 (4:3) gave the sim
        # an 18-deg-taller vertical FOV than the vehicle (sim-easier: saw the
        # gate bar/floor earlier when close). Same aspect at half resolution
        # keeps the Pi render cheap; Vision min-area thresholds scale by
        # actual image area, so detection behavior tracks automatically.
        self.cameraSurface = pygame.Surface((320, 160))
        _assets_dir = os.path.join(os.path.dirname(__file__), "assets")
        try:
            bg_img = pygame.image.load(os.path.join(_assets_dir, "BackgroundImage.jpg")).convert()
            h=480; w=int(bg_img.get_width()*(h/bg_img.get_height()))
            self.camera_background = pygame.transform.scale(bg_img, (w,h))
            self.camera_background_pano = pygame.Surface((w*2,h))
            self.camera_background_pano.blit(self.camera_background,(0,0)); self.camera_background_pano.blit(self.camera_background,(w,0))
        except pygame.error as e:
            print(f"Error loading background image: {e}")
            self.camera_background, self.camera_background_pano = None, None

        self.subMass = self.config.subMass
        self.subInertia_Z = self.config.subInertia_Z
        self.subInertia_Y = self.config.subInertia_Y
        self.subInertia_X = self.config.subInertia_X
        self.thrusterMaxForce = self.config.thrusterMaxForce
        self.force_gravity = self.subMass * self.config.gravity
        self.force_buoyancy = self.config.subVolume * self.config.waterDensity * self.config.gravity
        self.netBuoyancyForce = self.force_buoyancy - self.force_gravity

        self.submarineAI = submarine_ai
        self.ros_task_name  = 'Waiting...'
        self.ros_state_name = ''
        self.prequal_gate: Optional[PrequalGate] = None
        self.prequal_marker: Optional[PrequalMarker] = None
        self.resetSimulation()

    def resetSimulation(self):
        # Test-only reproducibility/start-pose knobs (defaults unchanged):
        #   ROBOSUB_SIM_SEED    int — seeds the course randomization
        #   ROBOSUB_START_X/Y   float — override the start position
        #   ROBOSUB_START_HDG   float — override the start heading (deg)
        seed = os.environ.get('ROBOSUB_SIM_SEED')
        if seed:
            random.seed(int(seed))
        self.prequal_gate = PrequalGate(
            x = self.prequal_config.GATE_X_POS, center_y = self.config.worldHeight / 2,
            z_top = self.prequal_config.GATE_DEPTH_METERS, width = self.prequal_config.GATE_WIDTH_METERS,
            height = self.prequal_config.GATE_OPENING_HEIGHT, color = self.prequal_config.GATE_COLOR
        )
        self.prequal_marker = PrequalMarker(
            x = self.prequal_config.MARKER_X_POS, y = self.config.worldHeight / 2,
            z_top = -self.prequal_config.POLE_ABOVE_SURFACE_METERS,
            z_bottom = self.config.worldDepth - 0.01,
            radius = self.prequal_config.MARKER_DIAMETER_METERS / 2, color = self.prequal_config.MARKER_COLOR
        )
        # World matches the mission (same env var mission.py reads): the FULL
        # course (default) replaces the orbit marker with a red/white slalom
        # lane; ROBOSUB_MISSION=orbit keeps the legacy marker course.
        self.slalom_poles = []
        if os.environ.get('ROBOSUB_MISSION', '').lower() != 'orbit':
            self.prequal_marker = None
            gate_x = self.prequal_config.GATE_X_POS
            spacing = 1.524            # white-red-white lateral spacing
            wiggle = spacing * 0.25
            # The whole lane sits laterally offset from the gate axis —
            # competition courses are staggered, so the vehicle must search
            # and translate to acquire the lane, not find it dead ahead.
            # Randomized per run; ROBOSUB_LANE_OFFSET forces a value (m,
            # + = world +y / port side when facing the course).
            off_env = os.environ.get('ROBOSUB_LANE_OFFSET')
            lane_offset = (float(off_env) if off_env
                           else random.uniform(-2.5, 2.5))
            print(f"[sim] slalom lane offset {lane_offset:+.2f} m", flush=True)
            last_y = self.config.worldHeight / 2 + lane_offset
            for i in range(3):
                sx = gate_x + 8.0 + i * 4.0
                y = last_y + random.uniform(-wiggle, wiggle) if i > 0 else last_y
                y = float(np.clip(y, spacing * 2, self.config.worldHeight - spacing * 2))
                last_y = y
                # Poles rise from the pool floor — bottom fixed at worldDepth,
                # height randomized so the top varies (previously the TOP
                # depth was randomized independent of height, which left the
                # bottoms floating 0.3-0.6m short of the floor instead of
                # planted in it).
                # RoboSub 2026 official: slalom pipes are 1 in PVC, 3 ft
                # (0.9 m) long, moored to the floor (buoyant). In shallow
                # water they stand from the floor; in deep water they float
                # moored with tops ~1.2 m down (mooring depth UNSPECIFIED
                # officially -- 1.2 m is an assumption near mission depth).
                pole_height = 0.9
                z_top = min(self.config.worldDepth - pole_height, 1.2)
                self.slalom_poles += [
                    SlalomPole(x=sx, y=y - spacing, z=z_top, height=pole_height, color=WHITE),
                    SlalomPole(x=sx, y=y,           z=z_top, height=pole_height, color=RED),
                    SlalomPole(x=sx, y=y + spacing, z=z_top, height=pole_height, color=WHITE),
                ]

        start_x = float(os.environ.get('ROBOSUB_START_X',
                                       self.prequal_config.START_X_POS))
        start_y = float(os.environ.get(
            'ROBOSUB_START_Y',
            self.config.worldHeight / 2 + random.uniform(-0.5, 0.5)))
        start_heading = float(os.environ.get('ROBOSUB_START_HDG', 0.0))
        start_depth = self.prequal_config.START_Z_POS

        self.subPhysics = SubmarinePhysicsState(
            x = start_x, y = start_y, z = start_depth,
            heading = start_heading, pitch = 0.0
        )
        if self.submarineAI:
            self.submarineAI.reset()
        self.startTime = time.time()
        self.running, self.paused = True, False
        self.lastThrusterCommands, self.last_imu_readings = ThrusterCommands(), MPU6050Readings()

    def worldToScreen(self, x, y):
        # (Remains the same)
        return int(x*self.scaleX+50), int((self.config.worldHeight-y)*self.scaleY+50)

    def handleInput(self):
        # (Remains the same)
        for event in pygame.event.get():
             if event.type == pygame.QUIT: self.running = False
             elif event.type == pygame.KEYDOWN:
                 if event.key == pygame.K_SPACE: self.paused = not self.paused
                 elif event.key == pygame.K_r: self.resetSimulation()

    def _thruster_force(self, cmd: float) -> float:
        """Epsilon-plant per-thruster force (N) from a normalized command.

        Measured quadratic (water session 1, S3 fit): F = a*(cmd%)^2 up to
        the edge of the measured data (40%), tangent-line above it — the
        quadratic's own extrapolation to 100% is unverified and OPTIMISTIC,
        the tangent is the conservative choice. Symmetric fwd/rev (S7 saw
        ~1.5x stronger reverse on the verticals — refine after water 2)."""
        a = self.config.thrustCurveQuadA
        lin = self.config.thrustCurveLinearizeAbove
        c = min(abs(cmd), 1.0) * 100.0
        if c <= lin:
            f = a * c * c
        else:
            f = a * lin * lin + 2.0 * a * lin * (c - lin)
        return math.copysign(f, cmd)

    def applyPhysics(self, dt, commands: ThrusterCommands):
        # (Remains the same - 6-DoF Physics)
        if self.config.epsilonPlant:
            # Actuation exactly as measured on the vehicle: quadratic
            # per-thruster curves; yaw arm fitted from S4 (torque, not
            # geometry — it folds in the unknown corner toe angle).
            f_hfl = self._thruster_force(commands.hfl)
            f_hfr = self._thruster_force(commands.hfr)
            f_hal = self._thruster_force(commands.hal)
            f_har = self._thruster_force(commands.har)
            f_vp = self._thruster_force(commands.vp)
            f_vs = self._thruster_force(commands.vs)
            yaw_arm = self.config.epsilonPlantYawArm
        else:
            f_hfl = commands.hfl * self.thrusterMaxForce
            f_hfr = commands.hfr * self.thrusterMaxForce
            f_hal = commands.hal * self.thrusterMaxForce
            f_har = commands.har * self.thrusterMaxForce
            f_vp = commands.vp * self.thrusterMaxForce
            f_vs = commands.vs * self.thrusterMaxForce
            yaw_arm = self.config.yawMomentArm
        cos_45 = 0.7071
        thrust_surge = (f_hfl + f_hfr + f_hal + f_har) * cos_45
        thrust_sway  = (f_hfl - f_hfr - f_hal + f_har) * cos_45
        thrust_heave = f_vp + f_vs
        # Yaw torque with an explicit moment arm (was an implicit 1.0 m —
        # ~3x optimistic on this hull; roll below has always used its arm).
        thrust_yaw  = (f_hfl - f_hfr + f_hal - f_har) * yaw_arm
        thrust_roll = (f_vp - f_vs) * self.config.rollMomentArm   # port−starboard differential → roll
        h_rad = math.radians(self.subPhysics.heading)
        cos_h, sin_h = math.cos(h_rad), math.sin(h_rad)
        vx_w, vy_w, vz_w = self.subPhysics.velocity_x, self.subPhysics.velocity_y, self.subPhysics.velocity_z
        vel_surge = vx_w * cos_h + vy_w * sin_h
        vel_sway  = -vx_w * sin_h + vy_w * cos_h
        vel_heave = vz_w
        drag_surge = -self.config.surgeDragCoeff * vel_surge * abs(vel_surge)
        drag_sway  = -self.config.swayDragCoeff * vel_sway * abs(vel_sway)
        drag_heave = -self.config.heaveDragCoeff * vel_heave * abs(vel_heave)
        drag_yaw   = -self.config.angularDragCoeff_Z * self.subPhysics.angular_velocity_z**2 * np.sign(self.subPhysics.angular_velocity_z)
        drag_roll  = -self.config.angularDragCoeff_X * self.subPhysics.angular_velocity_x**2 * np.sign(self.subPhysics.angular_velocity_x)
        drag_pitch = -self.config.angularDragCoeff_Y * self.subPhysics.angular_velocity_y**2 * np.sign(self.subPhysics.angular_velocity_y)
        # Rotate body-frame sway/heave thrust through roll so thrusters on an
        # inverted sub push the right way in the world frame (matters for the
        # style-roll maneuver; identity when roll = 0).
        r_rad = math.radians(self.subPhysics.roll)
        cos_r, sin_r = math.cos(r_rad), math.sin(r_rad)
        thrust_heave_w = thrust_sway * sin_r + thrust_heave * cos_r
        thrust_sway_w  = thrust_sway * cos_r - thrust_heave * sin_r
        # Passive pitch DOF (2026-07-06, sysid W5): no pitch actuators exist —
        # pitch is driven by the surge→bow-up coupling (the #1 measured
        # real-vs-sim gap), restored by the (deliberately small) righting
        # moment, damped by quadratic drag. Roll gets its righting moment the
        # same way — the style roll now costs real torque. Params are NOMINAL
        # priors until the S2/S5 fits land in sim_calibration.yaml.
        p_rad = math.radians(self.subPhysics.pitch)
        thrust_pitch = (self.config.surgePitchCoupling * thrust_surge
                        + self.config.surgePitchVelCoupling * vel_surge * abs(vel_surge))
        righting_pitch = -self.config.pitchRightingMoment * math.sin(p_rad)
        righting_roll = -self.config.rollRightingMoment * math.sin(r_rad)
        total_force_surge = thrust_surge + drag_surge
        total_force_sway  = thrust_sway_w + drag_sway
        total_force_heave = thrust_heave_w + drag_heave
        total_torque_yaw  = thrust_yaw + drag_yaw
        total_torque_roll = thrust_roll + drag_roll + righting_roll
        total_torque_pitch = thrust_pitch + righting_pitch + drag_pitch
        fx = total_force_surge * cos_h - total_force_sway * sin_h
        fy = total_force_surge * sin_h + total_force_sway * cos_h
        # Bow-up pitch tilts the surge thrust line: its world-vertical
        # component lifts the vehicle (z is down+), which is exactly why the
        # real sub needs extra down-thrust while surging.
        # Depth-dependent buoyancy (water session 1, 2026-07-07): compressible
        # volume makes the real sub near-neutral→negative by ~2 m (it sat on
        # the pool floor with zero thrust). z is down+, so lost buoyancy ADDS
        # to fz (sinks). Slope 0.0 = legacy incompressible behavior.
        net_buoyancy = (self.netBuoyancyForce
                        - self.config.buoyancyDepthSlope * max(self.subPhysics.z, 0.0))
        fz = total_force_heave - net_buoyancy - thrust_surge * math.sin(p_rad)
        ax = fx / self.subMass
        ay = fy / self.subMass
        az = fz / self.subMass
        angular_accel_z = total_torque_yaw  / self.subInertia_Z
        angular_accel_x = total_torque_roll / self.subInertia_X
        angular_accel_y = total_torque_pitch / self.subInertia_Y
        self.subPhysics.velocity_x += ax * dt
        self.subPhysics.velocity_y += ay * dt
        self.subPhysics.velocity_z += az * dt
        self.subPhysics.angular_velocity_z += angular_accel_z * dt
        self.subPhysics.angular_velocity_x += angular_accel_x * dt
        self.subPhysics.angular_velocity_y += angular_accel_y * dt
        imu_accel_surge = ax * cos_h + ay * sin_h
        imu_accel_sway = -ax * sin_h + ay * cos_h
        imu_accel_heave = az
        self.last_imu_readings = MPU6050Readings(
            accel_x=imu_accel_sway, accel_y=imu_accel_surge, accel_z=imu_accel_heave,
            gyro_z=self.subPhysics.angular_velocity_z,
            gyro_x=self.subPhysics.angular_velocity_x,
            gyro_y=self.subPhysics.angular_velocity_y,
        )
        prev_x = self.subPhysics.x
        self.subPhysics.x += self.subPhysics.velocity_x * dt
        self.subPhysics.y += self.subPhysics.velocity_y * dt
        self.subPhysics.z += self.subPhysics.velocity_z * dt
        self.subPhysics.heading = (self.subPhysics.heading + math.degrees(self.subPhysics.angular_velocity_z * dt)) % 360
        # Wrap roll to ±180 (was clipped to ±90, which made full barrel rolls
        # unrepresentable — needed for the style-roll maneuver).
        self.subPhysics.roll = ((self.subPhysics.roll
                                 + math.degrees(self.subPhysics.angular_velocity_x * dt)
                                 + 180.0) % 360.0) - 180.0
        # Pitch is passive and small; clip well short of ±90 so the camera
        # projection never degenerates (a real bow-up under surge is ~5-20°).
        self.subPhysics.pitch = float(np.clip(
            self.subPhysics.pitch + math.degrees(self.subPhysics.angular_velocity_y * dt),
            -75.0, 75.0))
        if abs(self.subPhysics.pitch) >= 75.0:
            self.subPhysics.angular_velocity_y = 0.0
        margin = 0.5
        self.subPhysics.x = np.clip(self.subPhysics.x, margin, self.config.worldWidth - margin)
        self.subPhysics.y = np.clip(self.subPhysics.y, margin, self.config.worldHeight - margin)
        self.subPhysics.z = np.clip(self.subPhysics.z, 0.0, self.config.worldDepth - 0.2)

        # Gate is a solid structure, not a checkpoint: crossing its X-plane
        # only counts as passing through the physical opening (between the
        # posts, under the bar). Anywhere else it's a wall — block the
        # crossing instead of letting the sub swim over the top or around
        # the sides.
        g = self.prequal_gate
        if g is not None:
            new_side = np.sign(self.subPhysics.x - g.x)
            old_side = np.sign(prev_x - g.x)
            if old_side != 0 and new_side != 0 and old_side != new_side:
                half_w = g.width / 2
                within_lateral = abs(self.subPhysics.y - g.center_y) <= half_w
                within_depth = self.subPhysics.z >= g.z_top
                if not (within_lateral and within_depth):
                    # Push back onto the ORIGINAL side (old_side is the sign
                    # of prev_x - g.x, so stepping +0.05 along it restores the
                    # approach side; the previous -0.05 teleported the vehicle
                    # THROUGH the wall).
                    self.subPhysics.x = g.x + old_side * 0.05
                    self.subPhysics.velocity_x = 0.0

    def project3D(self, world_pos: Tuple[float, float, float]) -> Optional[Tuple[int, int, float]]:
        dx,dy,dz = world_pos[0]-self.subPhysics.x, world_pos[1]-self.subPhysics.y, world_pos[2]-self.subPhysics.z
        h,p = math.radians(-self.subPhysics.heading), math.radians(-self.subPhysics.pitch)
        ch,sh,cp,sp = math.cos(h),math.sin(h),math.cos(p),math.sin(p)
        x_yaw, y_yaw = dx*ch-dy*sh, dx*sh+dy*ch
        cz,cy,cx = x_yaw*cp+dz*sp, x_yaw*sp-dz*cp, y_yaw
        if cz < 0.2: return None
        # Roll the image plane about the forward (cz) axis so the onboard
        # camera view actually tumbles during a barrel roll — previously roll
        # was tracked in physics but never reached the projection, so the
        # style-roll maneuver was invisible in the camera feed (looked like
        # the sub was just drifting rather than rolling).
        r = math.radians(self.subPhysics.roll)
        cr, sr = math.cos(r), math.sin(r)
        cx, cy = cx*cr - cy*sr, cx*sr + cy*cr
        w,h = self.cameraSurface.get_size()
        f = w/(2*math.tan(math.radians(self.config.cameraFov/2)))
        return int(w/2-f*(cx/cz)), int(h/2-f*(cy/cz)), math.hypot(dx,dy,dz)


    def generateCameraView(self):
        # --- MODIFIED: Marker drawing uses apparent width ---
        w,h = self.cameraSurface.get_size()
        if self.camera_background_pano:
             bg_w,bg_h = self.camera_background.get_size()
             x_off = (self.subPhysics.heading/360)*bg_w
             y_off = np.clip(((bg_h-h)/2)-(self.subPhysics.pitch*2.0), 0, bg_h-h)
             self.cameraSurface.blit(self.camera_background_pano, (x_off - bg_w, -y_off))
        else:
             self.cameraSurface.fill(WATER_COLOR)

        drawable = []
        if self.prequal_gate:
             g = self.prequal_gate
             half_w = g.width / 2
             pole_color = g.color
             bar_z = g.z_top
             # 2026 gate posts are 1.5 m long (gate floats; posts do NOT
             # reach the floor). Vision post-height cues depend on this.
             pole_z_bottom = min(self.config.worldDepth - 0.01,
                                 g.z_top + g.height)
             l_bar = self.project3D((g.x, g.center_y - half_w, bar_z))
             r_bar = self.project3D((g.x, g.center_y + half_w, bar_z))
             l_bot = self.project3D((g.x, g.center_y - half_w, pole_z_bottom))
             r_bot = self.project3D((g.x, g.center_y + half_w, pole_z_bottom))
             if l_bar and r_bar:
                 avg_dist = (l_bar[2] + r_bar[2]) / 2
                 drawable.append((avg_dist, 'line', GRAY, l_bar[:2], r_bar[:2], 5)) # Bar is GRAY
             if l_bar and l_bot:
                 avg_dist = (l_bar[2] + l_bot[2]) / 2
                 drawable.append((avg_dist, 'line', pole_color, l_bar[:2], l_bot[:2], 5)) # Pole is RED
             if r_bar and r_bot:
                 avg_dist = (r_bar[2] + r_bot[2]) / 2
                 drawable.append((avg_dist, 'line', pole_color, r_bar[:2], r_bot[:2], 5)) # Pole is RED

             # 2026 gate trim, HONEST COLORS (2026-07-07 W6): the real
             # divider plate is RED and the maker boxes are BLACK (left) and
             # RED (right, "Red-Right-Above") — red-keyed vision MUST cope
             # with red blobs that are not posts. The divider is rejected by
             # the pair height-ratio filter (0.61 m vs 1.5 m posts); the red
             # box by the aspect filter (square). Pairing scans ALL candidate
             # pairs so the divider sitting between the posts cannot mask the
             # true (L,R) pair.
             pc = self.prequal_config
             c_bar = self.project3D((g.x, g.center_y, bar_z))
             c_bot = self.project3D((g.x, g.center_y, bar_z + pc.GATE_DIVIDER_HEIGHT))
             if c_bar and c_bot:
                 avg_dist = (c_bar[2] + c_bot[2]) / 2
                 drawable.append((avg_dist, 'line', RED, c_bar[:2], c_bot[:2], 4))

             box_hw = pc.GATE_PICTURE_WIDTH / 2
             for side, box_color in ((-1, BLACK), (1, RED)):
                 by = g.center_y + side * 0.30 * half_w
                 btl = self.project3D((g.x, by - box_hw, bar_z + 0.05))
                 btr = self.project3D((g.x, by + box_hw, bar_z + 0.05))
                 bbr = self.project3D((g.x, by + box_hw, bar_z + 0.05 + pc.GATE_PICTURE_HEIGHT))
                 bbl = self.project3D((g.x, by - box_hw, bar_z + 0.05 + pc.GATE_PICTURE_HEIGHT))
                 if btl and btr and bbr and bbl:
                     avg_dist = (btl[2] + btr[2] + bbr[2] + bbl[2]) / 4
                     drawable.append((avg_dist, 'polygon', box_color,
                                      [btl[:2], btr[:2], bbr[:2], bbl[:2]], 0))

             pic_hw = pc.GATE_PICTURE_WIDTH / 2
             for side in (-1, 1):
                 py = g.center_y + side * pc.GATE_PICTURE_OFFSET_FRAC * half_w
                 tl = self.project3D((g.x, py - pic_hw, bar_z))
                 tr = self.project3D((g.x, py + pic_hw, bar_z))
                 br = self.project3D((g.x, py + pic_hw, bar_z + pc.GATE_PICTURE_HEIGHT))
                 bl = self.project3D((g.x, py - pic_hw, bar_z + pc.GATE_PICTURE_HEIGHT))
                 if tl and tr and br and bl:
                     avg_dist = (tl[2] + tr[2] + br[2] + bl[2]) / 4
                     drawable.append((avg_dist, 'polygon', GATE_PICTURE_COLOR,
                                      [tl[:2], tr[:2], br[:2], bl[:2]], 0))

        if self.prequal_marker:
             m = self.prequal_marker
             # Compute the silhouette tangent: perpendicular to the viewing
             # direction in XY so the cylinder outline is correct from any angle.
             dx = m.x - self.subPhysics.x
             dy = m.y - self.subPhysics.y
             dist_xy = math.hypot(dx, dy)
             if dist_xy > 0.01:
                 perp_x = -dy / dist_xy
                 perp_y =  dx / dist_xy
             else:
                 perp_x, perp_y = 0.0, 1.0

             tl = self.project3D((m.x + m.radius * perp_x, m.y + m.radius * perp_y, m.z_top))
             tr = self.project3D((m.x - m.radius * perp_x, m.y - m.radius * perp_y, m.z_top))
             bl = self.project3D((m.x + m.radius * perp_x, m.y + m.radius * perp_y, m.z_bottom))
             br = self.project3D((m.x - m.radius * perp_x, m.y - m.radius * perp_y, m.z_bottom))

             if tl and tr and bl and br:
                 avg_dist = (tl[2] + tr[2] + bl[2] + br[2]) / 4
                 poly = [tl[:2], tr[:2], br[:2], bl[:2]]
                 drawable.append((avg_dist, 'polygon', m.color, poly, 0))
             elif tl and bl:
                 avg_dist = (tl[2] + bl[2]) / 2
                 drawable.append((avg_dist, 'line', m.color, tl[:2], bl[:2], 4))

        # Slalom lane: bottom-anchored red/white poles (full-course world)
        for pole in getattr(self, 'slalom_poles', []):
             tp = self.project3D((pole.x, pole.y, pole.z))
             bp = self.project3D((pole.x, pole.y, pole.z + pole.height))
             if tp and bp:
                 drawable.append(((tp[2] + bp[2]) / 2, 'line', pole.color, tp[:2], bp[:2], 5))

        drawable.sort(key=lambda x: x[0], reverse=True)
        for d in drawable:
             if d[1]=='line': pygame.draw.line(self.cameraSurface, d[2], d[3], d[4], d[5])
             elif d[1]=='polygon': pygame.draw.polygon(self.cameraSurface, d[2], d[3], d[4])
             elif d[1]=='rect': pygame.draw.rect(self.cameraSurface, d[2], d[3])
        # --- END MODIFIED SECTION ---

    def render(self):
        # (Remains the same)
        self.screen.fill(LIGHT_BLUE)
        pygame.draw.rect(self.screen, BLACK, (40,40,int(self.config.worldWidth*self.scaleX+20),int(self.config.worldHeight*self.scaleY+20)), 2)
        if self.prequal_gate: g = self.prequal_gate; p1 = self.worldToScreen(g.x, g.center_y - g.width / 2); p2 = self.worldToScreen(g.x, g.center_y + g.width / 2); pygame.draw.line(self.screen, g.color, p1, p2, 4)
        if self.prequal_marker: m = self.prequal_marker; pos_2d = self.worldToScreen(m.x, m.y); radius_scaled = max(2, int(m.radius * self.scaleX)); pygame.draw.circle(self.screen, m.color, pos_2d, radius_scaled); pygame.draw.circle(self.screen, BLACK, pos_2d, radius_scaled, 1)
        for pole in getattr(self, 'slalom_poles', []):
            pos_2d = self.worldToScreen(pole.x, pole.y); pygame.draw.circle(self.screen, pole.color, pos_2d, 4); pygame.draw.circle(self.screen, BLACK, pos_2d, 4, 1)
        subPos = self.worldToScreen(self.subPhysics.x, self.subPhysics.y); hRad, cos_h, sin_h = math.radians(self.subPhysics.heading), math.cos(math.radians(self.subPhysics.heading)), math.sin(math.radians(self.subPhysics.heading)); pvc_s = self.config.submarineWidth*self.scaleX/2
        corners = [(-pvc_s,-pvc_s), (pvc_s,-pvc_s), (pvc_s,pvc_s), (-pvc_s,pvc_s)]; rotated = [(subPos[0]+dx*cos_h-dy*sin_h, subPos[1]-(dx*sin_h+dy*cos_h)) for dx,dy in corners]; pygame.draw.polygon(self.screen,YELLOW,rotated,4)
        box_w,box_l=0.127*self.scaleY/2,self.config.submarineLength*self.scaleX/2; box_corners=[(-box_l,-box_w),(box_l,-box_w),(box_l,box_w),(-box_l,box_w)]; rotated_box=[(subPos[0]+dx*cos_h-dy*sin_h,subPos[1]-(dx*sin_h+dy*cos_h)) for dx,dy in box_corners]; pygame.draw.polygon(self.screen,CONTROL_BOX_GRAY,rotated_box)
        arrow_pts = [(box_l,-box_w),(box_l,box_w),(box_l+0.2*self.scaleX,0)]; rotated_arrow=[(subPos[0]+dx*cos_h-dy*sin_h, subPos[1]-(dx*sin_h+dy*cos_h)) for dx,dy in arrow_pts]; pygame.draw.polygon(self.screen, YELLOW, rotated_arrow)
        # Roll gauge: the top-down view is a bird's-eye projection and can't
        # otherwise show rotation about the forward axis, so a barrel roll
        # was invisible here even though it was happening correctly in
        # physics. A small dial next to the sub makes it visible: the red
        # dot orbits the circle once per 360 degrees of roll.
        gauge_r = 18; gauge_cx, gauge_cy = subPos[0], subPos[1] - 45
        pygame.draw.circle(self.screen, WHITE, (int(gauge_cx), int(gauge_cy)), gauge_r, 1)
        roll_rad = math.radians(self.subPhysics.roll)
        dot_x = gauge_cx + gauge_r * math.sin(roll_rad)
        dot_y = gauge_cy - gauge_r * math.cos(roll_rad)
        pygame.draw.circle(self.screen, RED, (int(dot_x), int(dot_y)), 4)
        self._renderUi(); scaled_camera = pygame.transform.scale(self.cameraSurface, (400, 200)); self.screen.blit(scaled_camera, (self.width-420, 20)); pygame.draw.rect(self.screen, BLACK, (self.width-420, 20, 400, 200), 2); pygame.display.flip()

    def _drawThrusterBar(self, x, y, label, value):
        # (Remains the same)
        bar_w, bar_h = 25, 80; center_y = y + bar_h / 2; pygame.draw.rect(self.screen, GRAY, (x, y, bar_w, bar_h), 2)
        if value > 0: pygame.draw.rect(self.screen, GREEN, (x + 1, center_y - value * bar_h / 2, bar_w - 2, value * bar_h / 2))
        elif value < 0: pygame.draw.rect(self.screen, RED, (x + 1, center_y, bar_w - 2, abs(value) * bar_h / 2))
        pygame.draw.line(self.screen, BLACK, (x, center_y), (x + bar_w, center_y), 1); label_surf = self.smallFont.render(label, True, BLACK); self.screen.blit(label_surf, (x + bar_w / 2 - label_surf.get_width() / 2, y + bar_h + 5))

    def _renderUi(self):
        # (Remains the same)
        y = 20; title = self.font.render("Autonomous Submarine Simulator (AUV)", True, BLACK); self.screen.blit(title, (20,y)); y+=40
        speed = math.hypot(self.subPhysics.velocity_x, self.subPhysics.velocity_y)
        if self.submarineAI:
            task  = self.submarineAI.get_current_task_name()
            state = self.submarineAI.get_current_state_name()
        else:
            task  = self.ros_task_name
            state = self.ros_state_name
        stats=[f"Time: {time.time()-self.startTime:.1f}s", f"Task: {task}", f"State: {state}",
               f"Speed (XY): {speed:.2f} m/s", f"Vel Z: {self.subPhysics.velocity_z:.2f} m/s",
               f"Heading: {self.subPhysics.heading:.1f}°", f"Roll: {self.subPhysics.roll:.1f}°",
               f"Depth: {self.subPhysics.z:.2f} m"]
        for s in stats: self.screen.blit(self.smallFont.render(s,True,BLACK),(20,y)); y+=20
        y+=10; imu = self.last_imu_readings
        imu_stats=["IMU:",
                   f" Accel Y(surge): {imu.accel_y: .2f} m/s²",
                   f" Accel X(sway): {imu.accel_x: .2f} m/s²",
                   f" Accel Z(heave): {imu.accel_z: .2f} m/s²",
                   f" Gyro Z(yaw): {math.degrees(imu.gyro_z): .1f}°/s",
                   f" Gyro X(roll): {math.degrees(imu.gyro_x): .1f}°/s"]
        for s in imu_stats: self.screen.blit(self.smallFont.render(s,True,BLACK),(20,y)); y+=18
        y = self.height - 80; controls=["Controls:", "S - Start", "SPACE - Pause/Resume", "R - Reset", "Q - Quit"]
        for c in controls: self.screen.blit(self.smallFont.render(c,True,BLACK),(20,y)); y+=18

        # Prominent WAITING banner
        if task == 'WAITING':
            banner = self.font.render("WAITING  —  Press S to Start", True, (255, 200, 0))
            bx = (self.width - banner.get_width()) // 2
            by = (self.height - banner.get_height()) // 2
            pygame.draw.rect(self.screen, (30, 30, 30),
                             (bx - 10, by - 8, banner.get_width() + 20, banner.get_height() + 16))
            self.screen.blit(banner, (bx, by))
        tx,ty = self.width-420,350; self.screen.blit(self.smallFont.render("Thruster Output:",True,BLACK),(tx,ty)); ty+=25
        tc=self.lastThrusterCommands
        h_labels=[("HFL",tc.hfl),("HFR",tc.hfr),("HAL",tc.hal),("HAR",tc.har)]
        v_labels=[("VP",tc.vp),("VS",tc.vs)]
        for i,(l,v) in enumerate(h_labels): self._drawThrusterBar(tx+i*40,ty,l,v)
        for i,(l,v) in enumerate(v_labels): self._drawThrusterBar(tx+180+i*40,ty,l,v)

    def run(self):
        # (Remains the same)
        while self.running:
             dt = self.clock.tick(60) / 1000.0;
             if dt > 0.1: dt = 0.1
             self.handleInput()
             if self.paused:
                 self.render()
                 continue
             self.generateCameraView()
             camera_np = np.ascontiguousarray(
                 np.transpose(pygame.surfarray.array3d(self.cameraSurface), (1, 0, 2))[:, :, ::-1]
             )
             sensors = SensorSuite(camera_image=camera_np, depth=self.subPhysics.z,
                                   heading=self.subPhysics.heading, roll=self.subPhysics.roll,
                                   imu=self.last_imu_readings,
                                   velocity_x=self.subPhysics.velocity_x, velocity_y=self.subPhysics.velocity_y,
                                   velocity_z=self.subPhysics.velocity_z)
             thrusterCommands, vision_results = self.submarineAI.update(dt, sensors)
             self.lastThrusterCommands = thrusterCommands
             # --- Vision Debug Drawing (Remains the same) ---
             for pole in vision_results.red_blobs:
                  pygame.draw.rect(self.cameraSurface, ORANGE,
                                   (pole['min_x'], pole['min_y'], pole['width'], pole['height']), 1)
             gate_pair = vision_results.get_gate_pair()
             if gate_pair:
                 left_pole, right_pole = gate_pair
                 min_x = left_pole['min_x']
                 max_x = right_pole['max_x']
                 min_y = min(left_pole['min_y'], right_pole['min_y'])
                 max_y = max(left_pole['max_y'], right_pole['max_y'])
                 pygame.draw.rect(self.cameraSurface, YELLOW, (min_x, min_y, max_x - min_x, max_y - min_y), 1)
             best_pole = vision_results.get_best_pole()
             if best_pole:
                  pygame.draw.rect(self.cameraSurface, GREEN,
                                   (best_pole['min_x'], best_pole['min_y'], best_pole['width'], best_pole['height']), 2)
             self.applyPhysics(dt, thrusterCommands)
             self.render()
        pygame.quit()