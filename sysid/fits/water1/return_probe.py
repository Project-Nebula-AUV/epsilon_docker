import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
import math
import pygame
pygame.init()
import numpy as np
from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import Vision
from robosub.sub.control import load_params

sim = SubmarineSimulator(None, width=100, height=80)
g = sim.prequal_gate
params = load_params()
holder = {}
v = Vision(image_provider=lambda: holder.get("img"),
           min_pole_pixels=int(params["min_pixels_for_detection"]),
           min_gate_pixels=int(params["min_gate_pixels"]))
print("slalom poles:", [(round(p.x, 1), round(p.y, 2), p.color) for p in sim.slalom_poles][:6])
for (x, y) in ((30, 8.9), (28, 8.9), (25, 8.9), (30, 7.5), (20, 8.9), (16, 7.5)):
    sim.subPhysics.x, sim.subPhysics.y, sim.subPhysics.z = x, y, 1.55
    sim.subPhysics.heading = 180.0
    sim.subPhysics.roll = 0.0
    sim.subPhysics.pitch = 0.0
    sim.generateCameraView()
    holder["img"] = np.ascontiguousarray(np.transpose(
        pygame.surfarray.array3d(sim.cameraSurface), (1, 0, 2))[:, :, ::-1])
    v.update()
    posts = v.get_gate_post_blobs()
    pair = v.get_gate_pair()
    reds = [(round(b['center_x']), b['height'], b['width']) for b in v.red_blobs]
    exc = [(round(b['center_x']), b['height']) for b in v.red_blobs
           if b['height'] > b['width'] * 1.5 and v._is_slalom_red(b)]
    print(f"x={x} y={y}: reds={reds}")
    print(f"   posts(after excl)={[(round(b['center_x']), b['height']) for b in posts]} "
          f"slalom-excluded={exc} pair={'Y' if pair else 'NO'}")
