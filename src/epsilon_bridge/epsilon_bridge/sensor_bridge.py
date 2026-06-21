import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
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
    """

    def __init__(self):
        super().__init__('sensor_bridge')
        self.heading_sign = float(self.declare_parameter('heading_sign', 1.0).value)
        self.roll_sign = float(self.declare_parameter('roll_sign', 1.0).value)
        self.yaw_rate_sign = float(self.declare_parameter('yaw_rate_sign', 1.0).value)
        self.roll_rate_sign = float(self.declare_parameter('roll_rate_sign', 1.0).value)
        self.heading_offset_deg = float(self.declare_parameter('heading_offset_deg', 0.0).value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_heading = self.create_publisher(Float32, '/sensors/heading', qos)
        self.pub_roll = self.create_publisher(Float32, '/sensors/roll', qos)
        self.pub_depth = self.create_publisher(Float32, '/sensors/depth', qos)
        self.pub_imu = self.create_publisher(Imu, '/sensors/imu', qos)
        self.pub_vel = self.create_publisher(Twist, '/sensors/velocity', qos)

        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(Float32, '/depth', self.on_depth, 10)
        self.create_timer(0.05, self.on_velocity_timer)  # zeros, no DVL
        self.get_logger().info('sensor_bridge up (roll-rate=raw.y, yaw-rate=raw.z)')

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
        heading = (math.degrees(yaw) * self.heading_sign + self.heading_offset_deg) % 360.0
        roll = math.degrees(pitch) * self.roll_sign
        self.pub_heading.publish(Float32(data=float(heading)))
        self.pub_roll.publish(Float32(data=float(roll)))

    def on_depth(self, msg):
        self.pub_depth.publish(Float32(data=float(msg.data)))

    def on_velocity_timer(self):
        self.pub_vel.publish(Twist())


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
