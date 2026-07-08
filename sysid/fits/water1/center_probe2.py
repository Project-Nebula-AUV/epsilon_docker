import os, sys
os.environ["SDL_VIDEODRIVER"] = "dummy"
plant = sys.argv[1] if len(sys.argv) > 1 else "1"
os.environ["ROBOSUB_EPSILON_PLANT"] = plant
x0, y0, h0 = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
import math
import pygame
pygame.init()
import numpy as np
from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import SensorSuite, MPU6050Readings
from robosub.sub.submarine import Submarine
from robosub.sub.tasks.common_subtasks import CenterOnGateHalf, gate_half_center_px, _err_norm

sim = SubmarineSimulator(None, width=100, height=80)
g = sim.prequal_gate
sub = Submarine([])
st = CenterOnGateHalf(side='right')
sim.subPhysics.x, sim.subPhysics.y, sim.subPhysics.z = x0, y0, 1.55
sim.subPhysics.heading = h0
ctx = {'target_depth': 1.55, 'axis': h0}
dt = 1.0/18.2
t, last, nopair_t = 0.0, -1.0, 0.0
while t < 45.0:
    sim.generateCameraView()
    cam = np.ascontiguousarray(np.transpose(
        pygame.surfarray.array3d(sim.cameraSurface), (1, 0, 2))[:, :, ::-1])
    imu = MPU6050Readings()
    imu.gyro_z = sim.subPhysics.angular_velocity_z
    imu.gyro_x = sim.subPhysics.angular_velocity_x
    sensors = SensorSuite(camera_image=cam, depth=sim.subPhysics.z,
                          heading=sim.subPhysics.heading, roll=sim.subPhysics.roll,
                          imu=imu, velocity_x=sim.subPhysics.velocity_x,
                          velocity_y=sim.subPhysics.velocity_y,
                          velocity_z=sim.subPhysics.velocity_z)
    sub._latest_sensors = sensors
    sub.vision.update()
    status, cmds = st.tick(sub, dt, sensors, sub.vision, sim.config, ctx)
    sim.applyPhysics(dt, cmds)
    pair = sub.vision.get_gate_pair()
    if pair is None:
        nopair_t += dt
    if t - last >= 3.0 or status.name != 'RUNNING':
        sep = (abs(pair[1]['center_x']-pair[0]['center_x'])/160.0) if pair else -1
        d = g.x - sim.subPhysics.x
        print(f"t={t:5.1f} d={d:4.2f} y={sim.subPhysics.y:5.2f} hdg={sim.subPhysics.heading:6.1f} "
              f"pair={'Y' if pair else 'n'} sep={sep:5.2f} ok={st._ok} "
              f"blind={st._blind:4.1f} {status.name}")
        last = t
    if status.name != 'RUNNING':
        break
    t += dt
print(f"FINAL: {status.name} t={t:.1f} nopair_total={nopair_t:.1f}s")
