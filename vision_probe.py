#!/usr/bin/env python3
"""Probe: render one sim camera frame at the gate-search pose and dump what
Vision sees — red blobs, white blobs, slalom-red exclusions, gate pair."""
import os
os.environ['SDL_VIDEODRIVER'] = 'dummy'

import numpy as np
import pygame

from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import Vision

sim = SubmarineSimulator(submarine_ai=None)

for depth in (0.5, 1.0, 1.5, 1.8):
    sim.subPhysics.x, sim.subPhysics.y = 7.0, sim.config.worldHeight / 2
    sim.subPhysics.z, sim.subPhysics.heading = depth, 0.0
    sim.generateCameraView()
    arr = pygame.surfarray.array3d(sim.cameraSurface)   # (w,h,3) RGB
    frame_bgr = np.transpose(arr, (1, 0, 2))[:, :, ::-1].copy()
    v = Vision(image_provider=lambda: frame_bgr)
    v.update()
    excl = [f"h={b['height']}excl={v._is_slalom_red(b)}"
            for b in v.red_blobs if b['height'] > b['width'] * 1.5]
    print(f"z={depth}: gate={'FOUND' if v.get_gate_pair() else 'NONE '} "
          f"slalom={'FOUND' if v.get_slalom_gatelet() else 'NONE '} "
          f"tall_reds=[{', '.join(excl)}]")
