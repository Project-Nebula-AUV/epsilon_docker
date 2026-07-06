#!/usr/bin/env python3
import json
import math
import random

import pygame
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist, Vector3Stamped
from cv_bridge import CvBridge

from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import ThrusterCommands

# Depth-frame gravity magnitude for the synthesized /imu/gravity vector.
_G = 9.80665

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')
        self._bridge = CvBridge()
        self._commands = ThrusterCommands()

        # Match QoS with SubmarineNode
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Sensor-model calibration (2026-07-06, sysid W5): measured values live
        # in sysid/sim_calibration.yaml `sensors:` — a value there becomes the
        # DEFAULT of the matching parameter (CLI -p still wins). Missing file =
        # the nominal defaults below = pre-calibration behavior.
        self._cal = self._load_calibration_sensors()

        def _cp(name, nominal):
            return self.declare_parameter(name, self._cal.get(name, nominal)).value

        # Depth source (2026-07-06 — matches the vehicle's new architecture):
        #   truth  = ground-truth /sensors/depth (ideal mode, legacy default)
        #   esp32  = emulate the MS5837-on-Xiao chain: ~7 Hz filtered updates
        #            republished at 20 Hz ZOH (what nav sees via the
        #            esp32_depth driver + sensor_bridge on hardware)
        #   fused  = LEGACY marginal-MS5837-on-Pi emulation + external
        #            depth_fusion node (kept runnable; fuse_depth:=true maps
        #            here for back-compat)
        self.depth_mode = str(self.declare_parameter('depth_mode', 'truth').value)
        self.fuse_depth = bool(self.declare_parameter('fuse_depth', False).value)
        if self.fuse_depth:
            self.depth_mode = 'fused'
        # Legacy fused-emulation characteristics (bench, pre-ESP32):
        self.depth_attempt_hz = float(self.declare_parameter('depth_attempt_hz', 8.0).value)
        self.depth_success_prob = float(self.declare_parameter('depth_success_prob', 0.16).value)
        self.depth_corrupt_prob = float(self.declare_parameter('depth_corrupt_prob', 0.04).value)
        self.depth_noise_std = float(self.declare_parameter('depth_noise_std', 0.02).value)
        # ESP32-chain model (measured 2026-07-06, refit after S3):
        self.esp32_update_hz = float(_cp('esp32_update_hz', 7.2))
        self.esp32_noise_std = float(_cp('esp32_noise_std', 0.008))
        self.esp32_gap_prob = float(_cp('esp32_gap_prob', 0.002))   # per update: filter eating an outlier burst
        self.esp32_publish_hz = float(_cp('esp32_publish_hz', 20.0))
        self._esp_depth = None
        self._esp_vz = 0.0
        self._esp_accum = 0.0
        self._esp_pub_accum = 0.0
        self._esp_gap_left = 0.0
        # Attitude/IMU channel realism (nominal 0/off = ideal; calibration file
        # carries the measured a4-rest values):
        self.sensor_rate_hz = float(_cp('sensor_rate_hz', 0.0))    # 0 = every frame
        self.sensor_gap_prob = float(_cp('sensor_gap_prob', 0.0))  # per sample
        self.sensor_gap_s = float(_cp('sensor_gap_s', 0.45))
        self.heading_noise_std = float(_cp('heading_noise_std', 0.0))  # deg
        self.roll_noise_std = float(_cp('roll_noise_std', 0.0))        # deg
        self.imu_gyro_noise_std = float(_cp('imu_gyro_noise_std', 0.0))    # rad/s
        self.imu_accel_noise_std = float(_cp('imu_accel_noise_std', 0.0))  # m/s^2
        self.imu_gyro_zero_prob = float(_cp('imu_gyro_zero_prob', 0.0))    # corrupt read => zeroed group
        self.imu_accel_zero_prob = float(_cp('imu_accel_zero_prob', 0.0))
        self._sensor_accum = 0.0
        self._sensor_gap_left = 0.0
        # Hardware-faithful velocity: publish /sensors/velocity the way the
        # REAL vehicle senses it — x/y always zero (no DVL) and z from the
        # depth_fusion node's /depth_fusion/velocity_z (stale -> 0), instead
        # of physics ground truth. Combine with fuse_depth:=true to rehearse
        # the mission under genuine hardware sensing.
        self.hw_velocity = bool(self.declare_parameter('hw_velocity', False).value)
        self._fused_vz = 0.0
        self._fused_vz_t = None
        if self.hw_velocity:
            self.create_subscription(Float32, '/depth_fusion/velocity_z',
                                     self._fused_vz_cb, 10)
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        # Ground-truth trajectory + course-geometry log for post-run judging
        # (judge.py). On by default: the files are small and every sim run is
        # a test run.
        self.truth_log = bool(self.declare_parameter('truth_log', True).value)
        self._truth_file = None
        self._truth_t0 = None

        self._image_pub    = self.create_publisher(Image,             '/camera/image_raw', 10)
        self._depth_pub    = self.create_publisher(Float32,           '/sensors/depth',    self.qos)
        self._heading_pub  = self.create_publisher(Float32,           '/sensors/heading',  self.qos)
        self._roll_pub     = self.create_publisher(Float32,           '/sensors/roll',     self.qos)
        self._imu_pub      = self.create_publisher(Imu,               '/sensors/imu',      self.qos)
        self._velocity_pub = self.create_publisher(Twist,             '/sensors/velocity', self.qos)
        self._ctrl_pub     = self.create_publisher(String,            '/sim/control',      10)
        # Always-on ground-truth reference so fused depth can be scored in sim.
        self._true_depth_pub = self.create_publisher(Float32, '/sim/true_depth', 10)
        # Raw hardware-side topics, published only in legacy fused mode.
        if self.depth_mode == 'fused':
            self._raw_imu_pub  = self.create_publisher(Imu, '/imu', 10)
            self._gravity_pub  = self.create_publisher(Vector3Stamped, '/imu/gravity', 10)
            self._depth_raw_pub = self.create_publisher(Float32, '/depth_raw', 10)
            self._depth_attempt_accum = 0.0
        depth_desc = {
            'truth': 'ground-truth passthrough /sensors/depth',
            'esp32': 'ESP32-chain emulation (~%.1f Hz updates, %.0f Hz ZOH)'
                     % (self.esp32_update_hz, self.esp32_publish_hz),
            'fused': 'LEGACY fused via depth_fusion (emulated marginal MS5837)',
        }.get(self.depth_mode, 'UNKNOWN MODE %r' % self.depth_mode)
        self.get_logger().info(
            'simulator_node up (depth: %s; sensor rate: %s)'
            % (depth_desc,
               'every frame' if self.sensor_rate_hz <= 0
               else '%.1f Hz emulated' % self.sensor_rate_hz))

        self.create_subscription(Float32MultiArray, '/thruster_commands', self._thruster_cb, self.qos)
        self.create_subscription(String, '/sub/status',   self._status_cb, 10)
        self.create_subscription(String, '/sim/control',  self._ctrl_cb,   10)

        self.sim = SubmarineSimulator()

    def _load_calibration_sensors(self):
        """`sensors:` section of sysid/sim_calibration.yaml (empty if absent)."""
        import os
        import yaml
        path = os.environ.get('ROBOSUB_CALIBRATION',
                              '/home/robosub/robosub_ws/sysid/sim_calibration.yaml')
        try:
            with open(path) as f:
                cal = (yaml.safe_load(f) or {}).get('sensors') or {}
            if cal:
                print('[simulator_node] sensor calibration from %s: %s'
                      % (path, sorted(cal)), flush=True)
            return cal
        except FileNotFoundError:
            return {}
        except Exception as e:
            print('[simulator_node] calibration load FAILED (%s): %s' % (path, e),
                  flush=True)
            return {}

    def _fused_vz_cb(self, msg: Float32):
        self._fused_vz = float(msg.data)
        self._fused_vz_t = self.get_clock().now()

    def _thruster_cb(self, msg: Float32MultiArray):
        if len(msg.data) == 6:
            self._commands = ThrusterCommands(
                hfl=float(msg.data[0]), hfr=float(msg.data[1]),
                hal=float(msg.data[2]), har=float(msg.data[3]),
                vp=float(msg.data[4]),  vs=float(msg.data[5])
            )

    def _status_cb(self, msg: String):
        parts = msg.data.split('|', 1)
        self.sim.ros_task_name = parts[0]
        self.sim.ros_state_name = parts[1] if len(parts) > 1 else ''

    def _ctrl_cb(self, msg: String):
        cmd = msg.data
        if cmd == 'pause':
            self.sim.paused = True
            self._commands = ThrusterCommands()   # zero thrust while paused
        elif cmd == 'resume':
            self.sim.paused = False
        elif cmd == 'reset':
            self.sim.resetSimulation()
            self._commands = ThrusterCommands()
        elif cmd == 'quit':
            self.sim.running = False

    def publish_sensors(self, dt=0.0):
        p = self.sim.subPhysics
        imu = self.sim.last_imu_readings
        now = self.get_clock().now().to_msg()

        # Ground truth, always -- lets fused depth be scored against reality.
        self._true_depth_pub.publish(Float32(data=float(p.z)))

        if self.truth_log:
            self._write_truth(p)

        # Depth by source mode (see __init__).
        if self.depth_mode == 'fused':
            self._publish_emulated_depth_raw(p.z, dt)
        elif self.depth_mode == 'esp32':
            self._publish_esp32_depth(p.z, dt)
        else:
            self._depth_pub.publish(Float32(data=float(p.z)))

        # Attitude + IMU at the emulated hardware cadence (every frame when
        # sensor_rate_hz <= 0), with the measured noise floors and the
        # zeroed-group corrupt-read convention when calibrated.
        if self._sensor_tick(dt):
            heading = float(p.heading)
            roll = float(p.roll)
            if self.heading_noise_std > 0.0:
                heading = (heading + random.gauss(0.0, self.heading_noise_std)) % 360.0
            if self.roll_noise_std > 0.0:
                roll += random.gauss(0.0, self.roll_noise_std)
            self._heading_pub.publish(Float32(data=heading))
            self._roll_pub.publish(Float32(data=roll))

            imu_msg = Imu()
            imu_msg.header.stamp = now
            if random.random() >= self.imu_gyro_zero_prob:
                n = self.imu_gyro_noise_std
                imu_msg.angular_velocity.z = float(imu.gyro_z) + (random.gauss(0, n) if n > 0 else 0.0)
                imu_msg.angular_velocity.x = float(imu.gyro_x) + (random.gauss(0, n) if n > 0 else 0.0)
                imu_msg.angular_velocity.y = float(imu.gyro_y) + (random.gauss(0, n) if n > 0 else 0.0)
            # else: corrupt read -> whole gyro group zeroed (driver convention)
            if random.random() >= self.imu_accel_zero_prob:
                n = self.imu_accel_noise_std
                imu_msg.linear_acceleration.x = float(imu.accel_x) + (random.gauss(0, n) if n > 0 else 0.0)
                imu_msg.linear_acceleration.y = float(imu.accel_y) + (random.gauss(0, n) if n > 0 else 0.0)
                imu_msg.linear_acceleration.z = float(imu.accel_z) + (random.gauss(0, n) if n > 0 else 0.0)
            self._imu_pub.publish(imu_msg)

            if self.depth_mode == 'fused':
                self._publish_raw_imu(imu, p.roll, now)

        vel = Twist()
        if self.hw_velocity:
            # As the real vehicle senses it: x/y zero (no DVL), z from the
            # depth chain (esp32 driver's derivative, or fusion in legacy mode).
            if self.depth_mode == 'esp32':
                vel.linear.z = float(self._esp_vz)
            elif (self._fused_vz_t is not None
                    and (self.get_clock().now() - self._fused_vz_t).nanoseconds * 1e-9 < 1.0):
                vel.linear.z = self._fused_vz
        else:
            vel.linear.x, vel.linear.y, vel.linear.z = float(p.velocity_x), float(p.velocity_y), float(p.velocity_z)
        self._velocity_pub.publish(vel)

        camera_np = np.ascontiguousarray(np.transpose(pygame.surfarray.array3d(self.sim.cameraSurface), (1, 0, 2))[:, :, ::-1])
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(camera_np, encoding='bgr8'))

    def _write_truth(self, p):
        """Append the ground-truth pose to /tmp/sim_truth.csv and dump the
        course geometry to /tmp/sim_geom.json once (judge.py reads both)."""
        if self._truth_file is None:
            self._truth_file = open('/tmp/sim_truth.csv', 'w')
            self._truth_file.write('t,x,y,z,heading,roll,task,state\n')
            self._truth_t0 = float(self.get_clock().now().nanoseconds) * 1e-9
            geom = {'gate': None, 'poles': []}
            g = getattr(self.sim, 'prequal_gate', None)
            if g is not None:
                geom['gate'] = {'x': float(g.x), 'center_y': float(g.center_y),
                                'z_top': float(g.z_top), 'width': float(g.width),
                                'height': float(g.height)}
            for pl in getattr(self.sim, 'slalom_poles', []):
                c = getattr(pl, 'color', (0, 0, 0))
                geom['poles'].append({'x': float(pl.x), 'y': float(pl.y),
                                      'z': float(pl.z),
                                      'height': float(pl.height),
                                      'red': bool(c[0] > c[1] + 40)})
            with open('/tmp/sim_geom.json', 'w') as f:
                json.dump(geom, f)
        t = float(self.get_clock().now().nanoseconds) * 1e-9 - self._truth_t0
        self._truth_file.write('%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%s,%s\n' % (
            t, p.x, p.y, p.z, p.heading, p.roll,
            getattr(self.sim, 'ros_task_name', ''),
            getattr(self.sim, 'ros_state_name', '')))
        self._truth_file.flush()

    def _publish_raw_imu(self, imu, roll_deg, now):
        """Publish the hardware-side /imu + /imu/gravity the fusion consumes.

        Raw IMU convention (matches the vehicle): BODY frame with x=sway
        (starboard), y=surge (forward), z=heave (down); linear acceleration
        is kinematic (gravity-removed), gravity is the down direction
        expressed in the SAME body frame. Both must rotate together with
        roll (about the forward axis) — a real accelerometer is bolted to
        the hull. Then the fusion's projection a.g/|g| returns the true
        world vertical acceleration at ANY attitude:
            a_vert = (ax*cos r - az*sin r)(-sin r) + (ax*sin r + az*cos r)(cos r)
                   = az_world.
        (An earlier version published world-frame accel with tilted gravity;
        past 90 degrees of roll the projection FLIPPED SIGN, the fused
        estimate ran away and nav chased it to the surface mid-style-roll.)
        """
        r = math.radians(roll_deg)
        cr, sr = math.cos(r), math.sin(r)
        ax_w, ay_w, az_w = float(imu.accel_x), float(imu.accel_y), float(imu.accel_z)
        raw = Imu()
        raw.header.stamp = now
        raw.header.frame_id = self.frame_id
        raw.angular_velocity.x = float(imu.gyro_x)
        raw.angular_velocity.z = float(imu.gyro_z)
        raw.linear_acceleration.x = ax_w * cr - az_w * sr
        raw.linear_acceleration.y = ay_w
        raw.linear_acceleration.z = ax_w * sr + az_w * cr
        # Orientation quaternion (roll about x, pitch 0, yaw=heading) -- only
        # the fusion's fallback path uses it; the gravity vector is primary.
        y = math.radians(self.sim.subPhysics.heading)
        cr, sr, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(y / 2), math.sin(y / 2)
        raw.orientation.w = cr * cy
        raw.orientation.x = sr * cy
        raw.orientation.y = sr * sy
        raw.orientation.z = cr * sy
        self._raw_imu_pub.publish(raw)

        # Gravity (down) in the SAME rolled body frame: world (0,0,+G)
        # rotated by -r about the forward axis -> (-G sin r, 0, G cos r).
        g = Vector3Stamped()
        g.header.stamp = now
        g.header.frame_id = self.frame_id
        g.vector.x = -_G * sr
        g.vector.y = 0.0
        g.vector.z = _G * cr
        self._gravity_pub.publish(g)

    def _sensor_tick(self, dt):
        """True when the emulated attitude/IMU chain delivers a sample.

        rate <= 0 = every frame (ideal). Otherwise fire at sensor_rate_hz with
        the measured occasional long dropout (a4 rest: one 0.45 s gap/10 min).
        """
        if self.sensor_rate_hz <= 0.0:
            return True
        if self._sensor_gap_left > 0.0:
            self._sensor_gap_left -= dt
            return False
        self._sensor_accum += dt
        if self._sensor_accum < 1.0 / self.sensor_rate_hz:
            return False
        self._sensor_accum = 0.0
        if self.sensor_gap_prob > 0.0 and random.random() < self.sensor_gap_prob:
            self._sensor_gap_left = self.sensor_gap_s
        return True

    def _publish_esp32_depth(self, true_z, dt):
        """Emulate what NAV SEES from the ESP32 depth chain (2026-07-06 HW):
        the esp32_depth driver filters a ~7 Hz JSON stream and republishes the
        latest accepted depth at 20 Hz (zero-order hold), plus a low-passed
        vertical velocity. Occasional outliers are REJECTED by the driver's
        filter, which downstream just experiences as a short update gap --
        modeled by esp32_gap_prob. Measured bench 2026-07-06; refit after S3."""
        if self._esp_gap_left > 0.0:
            self._esp_gap_left -= dt
        else:
            self._esp_accum += dt
            interval = 1.0 / self.esp32_update_hz
            if self._esp_accum >= interval:
                self._esp_accum = 0.0
                prev = self._esp_depth
                self._esp_depth = true_z + random.gauss(0.0, self.esp32_noise_std)
                if prev is not None:
                    v = (self._esp_depth - prev) * self.esp32_update_hz
                    v = max(-1.5, min(1.5, v))
                    self._esp_vz += 0.5 * (v - self._esp_vz)   # driver's low-pass
                if random.random() < self.esp32_gap_prob:
                    self._esp_gap_left = random.uniform(0.3, 0.8)
        self._esp_pub_accum += dt
        if self._esp_depth is not None and self._esp_pub_accum >= 1.0 / self.esp32_publish_hz:
            self._esp_pub_accum = 0.0
            self._depth_pub.publish(Float32(data=float(self._esp_depth)))

    def _publish_emulated_depth_raw(self, true_z, dt):
        """Emulate the marginal MS5837: sparse, noisy, occasionally corrupt.

        Runs discrete sensor "attempts" at depth_attempt_hz; each attempt
        either yields a near-truth reading, a wildly-corrupt one (drawn from
        the failure modes seen on the bus), or nothing (a dropped read). This
        reproduces the ~1 Hz effective rate, multi-second gaps, and garbage
        that the innovation gate must survive.
        """
        self._depth_attempt_accum += dt
        interval = 1.0 / self.depth_attempt_hz
        while self._depth_attempt_accum >= interval:
            self._depth_attempt_accum -= interval
            r = random.random()
            if r < self.depth_success_prob:
                z = true_z + random.gauss(0.0, self.depth_noise_std)
                self._depth_raw_pub.publish(Float32(data=float(z)))
            elif r < self.depth_success_prob + self.depth_corrupt_prob:
                # Corruption modes observed on the bus, each drawn with spread
                # so two successive corrupt reads (nearly) never match -- real
                # bus garbage is scattered, so it must not accidentally form a
                # 2-read consensus in the gate.
                mode = random.random()
                if mode < 0.35:            # bus stuck low -> ~0
                    z = random.uniform(-0.05, 0.05)
                elif mode < 0.7:           # wild positive (bit-shift / float garbage)
                    z = random.uniform(20.0, 300.0)
                elif mode < 0.85:          # negative garbage
                    z = random.uniform(-12.0, -3.0)
                else:                      # plausible-looking offset
                    z = true_z + random.uniform(1.0, 5.0)
                self._depth_raw_pub.publish(Float32(data=float(z)))
            # else: dropped read -- publish nothing.

    def publish_sim_control(self, command: str):
        self._ctrl_pub.publish(String(data=command))

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    sim = node.sim
    clock = pygame.time.Clock()

    try:
        while sim.running and rclpy.ok():
            dt = clock.tick(60) / 1000.0
            if dt > 0.1: dt = 0.1
            rclpy.spin_once(node, timeout_sec=0)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    node.publish_sim_control('quit')
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        node.publish_sim_control('pause' if not sim.paused else 'resume')
                    elif event.key == pygame.K_s:
                        node.publish_sim_control('start')
                    elif event.key == pygame.K_r:
                        node.publish_sim_control('reset')
                    elif event.key == pygame.K_q:
                        node.publish_sim_control('quit')

            if not sim.paused:
                sim.generateCameraView()
                sim.applyPhysics(dt, node._commands)
                sim.lastThrusterCommands = node._commands
                node.publish_sensors(dt)
                sim.render()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__': main()