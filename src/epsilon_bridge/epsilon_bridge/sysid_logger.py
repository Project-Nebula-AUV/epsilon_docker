import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Float64MultiArray, Bool, String


def _quat_to_euler(w, x, y, z):
    """Quaternion -> (roll, pitch, yaw) rad, Tait-Bryan xyz (same math as
    sensor_bridge). On this vehicle's 90deg-rotated BNO055 mount the published
    nav 'roll' is this euler PITCH; euler ROLL is the presumptive body-pitch
    channel (bow-up) -- confirm channel+sign with the lift-bow hand test."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class SysidLogger(Node):
    """Log every sysid-relevant stream to CSVs + JPEGs under run_dir.

    Files (first column = wall receive time, s, this node's clock):
      imu.csv      t,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,eroll_deg,epitch_deg,eyaw_deg
                   (raw /imu; zeroed fields = corrupt read, treat as missing;
                    e* = euler of the raw quat, logged for convenience --
                    eroll is the presumptive BODY-PITCH channel on this mount)
      gravity.csv  t,gx,gy,gz            (raw /imu/gravity, body frame, points UP)
      depth_raw.csv t,depth_m            (every MS5837 read the driver accepted)
      fused.csv    t,depth_m,vz_ms,innov_m,stale
                   (innov/stale sampled at their latest values per fused tick)
      attitude.csv t,heading_deg,roll_deg (sensor_bridge nav channels)
      cmd.csv      t,t0,t1,t2,t3,t4,t5   (every /thrust_control actually sent --
                   in a DRY run this file IS the verification artifact)
      markers.csv  t,text                (runner step transitions + hand marks)
      frames/f<t>.jpg                    (camera at jpeg_hz, bgr8 reshaped)

    Writes are line-buffered appends; a crash loses at most the last line.
    """

    def __init__(self):
        super().__init__('sysid_logger')
        run_dir = str(self.declare_parameter('run_dir', '').value)
        self.jpeg_hz = float(self.declare_parameter('jpeg_hz', 4.0).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 85).value)
        if not run_dir:
            raise RuntimeError('run_dir param is required')
        self.run_dir = run_dir
        self.frames_dir = os.path.join(run_dir, 'frames')
        os.makedirs(self.frames_dir, exist_ok=True)

        self._files = {}
        self._open('imu', 't,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,eroll_deg,epitch_deg,eyaw_deg')
        self._open('gravity', 't,gx,gy,gz')
        self._open('depth_raw', 't,depth_m')
        self._open('fused', 't,depth_m,vz_ms,innov_m,stale')
        self._open('attitude', 't,heading_deg,roll_deg')
        self._open('cmd', 't,t0,t1,t2,t3,t4,t5')
        self._open('markers', 't,text')
        self._open('temp', 't,temp_c')   # F14: esp32 sensor temperature

        self._innov = float('nan')
        self._stale = -1
        self._vz = float('nan')
        self._heading = float('nan')
        self._last_jpeg_t = 0.0
        self._n_frames = 0
        self._counts = {}

        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(Vector3Stamped, '/imu/gravity', self.on_gravity, 10)
        self.create_subscription(Float32, '/depth_raw', self.on_depth_raw, 10)
        self.create_subscription(Float32, '/sensors/depth', self.on_fused, qos_be)
        self.create_subscription(Float32, '/depth_fusion/velocity_z', self.on_vz, 10)
        self.create_subscription(Float32, '/depth_fusion/innovation', self.on_innov, 10)
        self.create_subscription(Bool, '/depth_fusion/stale', self.on_stale, 10)
        # ESP32 depth path (2026-07-06): same latest-value state vars, the
        # driver's topics instead of fusion's. Only one source runs per launch.
        self.create_subscription(Float32, '/esp32_depth/velocity_z', self.on_vz, 10)
        self.create_subscription(Bool, '/esp32_depth/stale', self.on_stale, 10)
        self.create_subscription(Float32, '/esp32_depth/temperature', self.on_temp, 10)
        self.create_subscription(Float32, '/sensors/heading', self.on_heading, qos_be)
        self.create_subscription(Float32, '/sensors/roll', self.on_roll, qos_be)
        self.create_subscription(Float64MultiArray, '/thrust_control', self.on_cmd, 10)
        self.create_subscription(String, '/sysid/marker', self.on_marker, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_be)
        self.create_timer(5.0, self.on_status)
        self.get_logger().info('sysid_logger -> %s (jpeg %.1f Hz q%d)'
                               % (run_dir, self.jpeg_hz, self.jpeg_quality))

    def _open(self, name, header):
        f = open(os.path.join(self.run_dir, name + '.csv'), 'a', buffering=1)
        if f.tell() == 0:
            f.write(header + '\n')
        self._files[name] = f

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _w(self, name, line):
        self._files[name].write(line + '\n')
        self._counts[name] = self._counts.get(name, 0) + 1

    def on_imu(self, m):
        q, g, a = m.orientation, m.angular_velocity, m.linear_acceleration
        er = ep = ey = float('nan')
        if (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) > 1e-6:
            r, p, y = _quat_to_euler(q.w, q.x, q.y, q.z)
            er, ep, ey = math.degrees(r), math.degrees(p), math.degrees(y)
        self._w('imu', '%.4f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f'
                % (self._t(), q.w, q.x, q.y, q.z, g.x, g.y, g.z, a.x, a.y, a.z, er, ep, ey))

    def on_gravity(self, m):
        v = m.vector
        self._w('gravity', '%.4f,%.4f,%.4f,%.4f' % (self._t(), v.x, v.y, v.z))

    def on_depth_raw(self, m):
        self._w('depth_raw', '%.4f,%.4f' % (self._t(), m.data))

    def on_temp(self, m):
        self._w('temp', '%.4f,%.2f' % (self._t(), m.data))

    def on_fused(self, m):
        self._w('fused', '%.4f,%.4f,%.4f,%.4f,%d'
                % (self._t(), m.data, self._vz, self._innov, self._stale))

    def on_vz(self, m):
        self._vz = m.data

    def on_innov(self, m):
        self._innov = m.data

    def on_stale(self, m):
        self._stale = int(m.data)

    def on_heading(self, m):
        self._heading = m.data

    def on_roll(self, m):
        self._w('attitude', '%.4f,%.2f,%.2f' % (self._t(), self._heading, m.data))

    def on_cmd(self, m):
        if len(m.data) < 6:
            return
        self._w('cmd', '%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f'
                % ((self._t(),) + tuple(m.data[:6])))

    def on_marker(self, m):
        self._w('markers', '%.4f,"%s"' % (self._t(), m.data.replace('"', "'")))

    def on_image(self, m):
        t = self._t()
        if t - self._last_jpeg_t < 1.0 / self.jpeg_hz:
            return
        self._last_jpeg_t = t
        try:
            import cv2
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            cv2.imwrite(os.path.join(self.frames_dir, 'f%.3f.jpg' % t), img,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            self._n_frames += 1
        except Exception as e:
            self.get_logger().warn('jpeg write failed: %s' % e,
                                   throttle_duration_sec=10.0)

    def on_status(self):
        c = self._counts
        self.get_logger().info(
            'logged: imu %d, grav %d, raw %d, fused %d, att %d, cmd %d, mark %d, jpg %d'
            % (c.get('imu', 0), c.get('gravity', 0), c.get('depth_raw', 0),
               c.get('fused', 0), c.get('attitude', 0), c.get('cmd', 0),
               c.get('markers', 0), self._n_frames))


def main(args=None):
    rclpy.init(args=args)
    node = SysidLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for f in node._files.values():
            try:
                f.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
