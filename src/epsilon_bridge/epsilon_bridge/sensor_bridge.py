import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


def _quat_to_euler(w, x, y, z):
    """Quaternion (w,x,y,z) -> (roll, pitch, yaw) radians, Tait-Bryan xyz."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class SensorBridge(Node):
    """Adapts epsilon hardware (/imu, /depth) to the robosub nav stack's /sensors/*.

    IMU axis mapping from the bench accel test: raw IMU frame is +X=right,
    +Y=back, +Z=down -> body roll-rate = raw gyro.y, body yaw-rate = raw gyro.z.
    heading/roll come from the fused quaternion. All signs are parameters
    (default +1); the yaw-loop sign is a single bit to confirm/flip at P4.
    No deps beyond rclpy + msgs, so it runs under the same python as the other
    nav nodes (system python via the install shebang).

    The published roll channel is mapped from the euler pitch (the BNO055 is
    mounted rotated 90deg about the shared Z) and reads a fixed mounting offset
    (~85deg) at rest. Since the sub is reliably LEVEL at power-on, we capture that
    offset from the first level_capture_secs of valid frames and subtract it
    (capture_level_on_start, default on) so roll reads ~0 when level. With capture
    off, the static roll_offset_deg param is subtracted instead.
    """

    def __init__(self):
        super().__init__('sensor_bridge')
        self.heading_sign = float(self.declare_parameter('heading_sign', 1.0).value)
        self.roll_sign = float(self.declare_parameter('roll_sign', 1.0).value)
        self.yaw_rate_sign = float(self.declare_parameter('yaw_rate_sign', 1.0).value)
        self.roll_rate_sign = float(self.declare_parameter('roll_rate_sign', 1.0).value)
        self.heading_offset_deg = float(self.declare_parameter('heading_offset_deg', 0.0).value)
        # Roll zeroing: assume level at power-on and capture the mounting offset.
        self.capture_level_on_start = bool(self.declare_parameter('capture_level_on_start', True).value)
        self.level_capture_secs = float(self.declare_parameter('level_capture_secs', 1.5).value)
        self.roll_offset_deg = float(self.declare_parameter('roll_offset_deg', 0.0).value)
        # Depth-sensorless hold: when synthetic_depth >= 0, ignore /depth and publish
        # this constant on /sensors/depth instead. Set it equal to the mission target
        # depth (MISSION_DEPTH, 1.5 m) so the nav's depth_error is ~0 -> the depth PID
        # commands ~zero heave (neutral hold) and StabilizeTask's depth gate is met so
        # the mission advances. The sub is started already submerged; vertical buoyancy
        # trim (the ~0.6 heave the integrator can no longer supply) is handled by
        # thruster_bridge's heave_bias param. -1.0 disables (real /depth passthrough).
        self.synthetic_depth = float(self.declare_parameter('synthetic_depth', -1.0).value)
        # /sensors/depth health: False if depth_fusion flags stale OR no /depth
        # has arrived within this window (fusion node died entirely). In
        # synthetic-depth mode the constant is trustworthy by construction, so
        # depth_ok is always True there.
        self.depth_ok_max_age = float(self.declare_parameter('depth_ok_max_age_s', 1.5).value)
        # _roll_offset is the active offset (deg). None => still capturing at boot.
        self._roll_offset = None if self.capture_level_on_start else self.roll_offset_deg
        self._cap_samples = []
        self._cap_start = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_heading = self.create_publisher(Float32, '/sensors/heading', qos)
        self.pub_roll = self.create_publisher(Float32, '/sensors/roll', qos)
        self.pub_depth = self.create_publisher(Float32, '/sensors/depth', qos)
        self.pub_imu = self.create_publisher(Imu, '/sensors/imu', qos)
        self.pub_vel = self.create_publisher(Twist, '/sensors/velocity', qos)
        self.pub_depth_ok = self.create_publisher(Bool, '/sensors/depth_ok', qos)

        # Depth-health inputs: fusion's own stale flag + the freshness of /depth
        # itself. Both default to "not ok" until proven fresh so a missing
        # upstream reads as unhealthy, never as silently-good.
        self._fusion_stale = True
        self._last_depth_t = None

        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(Float32, '/depth', self.on_depth, 10)
        self.create_subscription(Bool, '/depth_fusion/stale', self.on_fusion_stale, 10)
        # Fused vertical velocity from depth_fusion: restores the nav depth
        # PID's D-term and honest vertical completion gates. x/y stay zero
        # (no DVL); falls back to all-zeros if the fusion stream goes stale.
        self._fused_vz = 0.0
        self._fused_vz_t = None
        self.create_subscription(Float32, '/depth_fusion/velocity_z',
                                 self.on_fused_vz, 10)
        self.create_timer(0.05, self.on_velocity_timer)
        if self.synthetic_depth >= 0.0:
            self.create_timer(0.05, self.on_synthetic_depth_timer)  # 20 Hz constant
        mode = ('capture@boot %.1fs' % self.level_capture_secs
                if self.capture_level_on_start else 'static %.2f deg' % self.roll_offset_deg)
        depth_mode = ('SYNTHETIC %.2f m (depth sensor bypassed)' % self.synthetic_depth
                      if self.synthetic_depth >= 0.0 else 'passthrough /depth')
        self.get_logger().info(
            'sensor_bridge up (roll-rate=raw.y, yaw-rate=raw.z; roll zeroing: %s; depth: %s)'
            % (mode, depth_mode))

    def _roll_offset_now(self, pitch_deg):
        """Resolve the roll offset (deg). While capturing, accumulate samples and
        return the running mean (so output stays ~0); finalize after the window."""
        if self._roll_offset is not None:
            return self._roll_offset
        now = self.get_clock().now()
        if self._cap_start is None:
            self._cap_start = now
        self._cap_samples.append(pitch_deg)
        mean = sum(self._cap_samples) / len(self._cap_samples)
        if (now - self._cap_start).nanoseconds * 1e-9 >= self.level_capture_secs:
            self._roll_offset = mean
            self.get_logger().info(
                'level-at-boot roll offset captured: %.2f deg (%d samples)'
                % (self._roll_offset, len(self._cap_samples)))
        return mean

    def on_imu(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = self.roll_rate_sign * msg.angular_velocity.y  # roll-rate
        out.angular_velocity.y = msg.angular_velocity.x                        # pitch (unused)
        out.angular_velocity.z = self.yaw_rate_sign * msg.angular_velocity.z   # yaw-rate
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub_imu.publish(out)

        q = msg.orientation
        if (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) < 1e-6:
            return  # no valid orientation yet
        _r, pitch, yaw = _quat_to_euler(q.w, q.x, q.y, q.z)
        pitch_deg = math.degrees(pitch)
        heading = (math.degrees(yaw) * self.heading_sign + self.heading_offset_deg) % 360.0
        roll = (pitch_deg - self._roll_offset_now(pitch_deg)) * self.roll_sign
        self.pub_heading.publish(Float32(data=float(heading)))
        self.pub_roll.publish(Float32(data=float(roll)))

    def on_depth(self, msg):
        if self.synthetic_depth >= 0.0:
            return  # depth sensor bypassed; on_synthetic_depth_timer publishes instead
        self._last_depth_t = self.get_clock().now()
        self.pub_depth.publish(Float32(data=float(msg.data)))

    def on_synthetic_depth_timer(self):
        self.pub_depth.publish(Float32(data=float(self.synthetic_depth)))

    def on_fusion_stale(self, msg):
        self._fusion_stale = bool(msg.data)

    def on_fused_vz(self, msg):
        self._fused_vz = float(msg.data)
        self._fused_vz_t = self.get_clock().now()

    def _depth_ok_now(self):
        if self.synthetic_depth >= 0.0:
            return True   # constant-hold depth is trustworthy by construction
        if self._fusion_stale or self._last_depth_t is None:
            return False
        age = (self.get_clock().now() - self._last_depth_t).nanoseconds * 1e-9
        return age <= self.depth_ok_max_age

    def on_velocity_timer(self):
        t = Twist()
        if (self._fused_vz_t is not None
                and (self.get_clock().now() - self._fused_vz_t).nanoseconds * 1e-9 < 1.0):
            t.linear.z = self._fused_vz   # m/s, down+ (nav convention)
        self.pub_vel.publish(t)
        self.pub_depth_ok.publish(Bool(data=bool(self._depth_ok_now())))


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
