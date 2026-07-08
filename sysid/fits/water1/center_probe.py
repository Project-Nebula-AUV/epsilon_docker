import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["ROBOSUB_EPSILON_PLANT"] = "1"
import math
import pygame
pygame.init()
import numpy as np
from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import SensorSuite, MPU6050Readings
from robosub.sub.submarine import Submarine
from robosub.sub.tasks.common_subtasks import CenterOnGateHalf

sim = SubmarineSimulator(None, width=100, height=80)
g = sim.prequal_gate
sub = Submarine([])           # ctrl + vision holder
st = CenterOnGateHalf(side='right')

sim.subPhysics.x = g.x - 4.5
sim.subPhysics.y = g.center_y - 0.6
sim.subPhysics.z = 1.55
sim.subPhysics.heading = 0.0

ctx = {'target_depth': 1.55, 'axis': 0.0}
dt = 1.0/18.2
t = 0.0
last_print = -1.0
status = None
while t < 60.0:
    sim.generateCameraView()
    cam = np.ascontiguousarray(np.transpose(
        pygame.surfarray.array3d(sim.cameraSurface), (1, 0, 2))[:, :, ::-1])
    imu = MPU6050Readings()
    imu.gyro_z = math.radians(0.0) + sim.subPhysics.angular_velocity_z
    imu.gyro_x = sim.subPhysics.angular_velocity_x
    sensors = SensorSuite(camera_image=cam, depth=sim.subPhysics.z,
                          heading=sim.subPhysics.heading, roll=sim.subPhysics.roll,
                          imu=imu,
                          velocity_x=sim.subPhysics.velocity_x,
                          velocity_y=sim.subPhysics.velocity_y,
                          velocity_z=sim.subPhysics.velocity_z)
    sub._latest_sensors = sensors
    sub.vision.update()
    status, cmds = st.tick(sub, dt, sensors, sub.vision, sim.config, ctx)
    sim.applyPhysics(dt, cmds)
    if t - last_print >= 2.0 or status.name != 'RUNNING':
        pair = sub.vision.get_gate_pair()
        sep = (abs(pair[1]['center_x']-pair[0]['center_x'])/160.0) if pair else -1
        from robosub.sub.tasks.common_subtasks import gate_half_center_px, _err_norm
        err = _err_norm(gate_half_center_px(pair, 'right'), 320) if pair else float('nan')
        print(f"t={t:5.1f} x={sim.subPhysics.x:5.2f} y={sim.subPhysics.y:5.2f} "
              f"z={sim.subPhysics.z:4.2f} hdg={sim.subPhysics.heading:6.1f} "
              f"pair={'Y' if pair else 'n'} err={err:+.3f} sep={sep:.2f} "
              f"pxrate={sub.ctrl.pixel_rate():+.3f} ok={st._ok} {status.name}")
        last_print = t
    if status.name != 'RUNNING':
        break
    t += dt
print("FINAL:", status.name, "t=%.1f" % t)
