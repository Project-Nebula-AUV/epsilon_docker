import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

import time

import board
import adafruit_bno055


class ImuInput(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters so the frame and rate can be retuned without code changes.
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.period = 1.0 / float(rate_hz)

        self.publisher = self.create_publisher(Imu, '/imu', 10)
        # Measured local "up" in the sensor body frame, from the BNO055's
        # own fusion. Lets consumers (depth_fusion) project accelerations
        # onto the true vertical no matter how the IMU is mounted, without
        # trusting quaternion axis/ordering conventions.
        self.gravity_publisher = self.create_publisher(
            Vector3Stamped, '/imu/gravity', 10)

        self.sensor = None
        self._last_connect_attempt = 0.0
        # Transient-vs-dead discrimination: on this bus a large fraction of
        # transactions fail while the sensor itself is fine, and a full
        # reconnect (chip-id check + mode setup, many transactions + ~650 ms
        # settle) usually fails too -- so a reconnect-on-first-error policy
        # turns one glitch into seconds of outage. Skip the cycle instead,
        # and only reinitialize after a long unbroken run of failures.
        self._consec_failures = 0
        self._reconnect_after = 60  # ~3 s of solid failures at 20 Hz

        # NOTE: the I2C bus MUST be clocked at 10 kHz for this sensor. At
        # 100 kHz the BNO055's clock-stretching wedges the kernel I2C
        # transfer (process stuck in 'D'/i2c_dw_xfer with no Python
        # exception, so it can't be caught here). The bus speed is set at the
        # system level (dtparam i2c_arm_baudrate=10000 / config.txt), not in
        # this driver.

        # Try to bring the sensor up now; if it isn't ready yet (the BNO055
        # needs ~650 ms after power-on) we keep retrying from the timer so the
        # node never crashes and comes good on its own.
        self.connect()

        self.timer = self.create_timer(self.period, self.timer_callback)
        self.get_logger().info('BNO055 IMU node started')

    def connect(self):
        """(Re)initialise the BNO055 over I2C. Returns True on success.

        Never raises -- on failure we log and let the timer retry, so a cold
        boot or a transient I2C glitch can't take the node down.
        """
        self._last_connect_attempt = time.monotonic()
        try:
            i2c = board.I2C()
            sensor = adafruit_bno055.BNO055_I2C(i2c)
            # Touch a register so a missing/not-ready sensor fails here rather
            # than on the first publish.
            _ = sensor.temperature
            self.sensor = sensor
            self.get_logger().info('BNO055 connected (NDOF)')
            return True
        except (OSError, ValueError, RuntimeError) as e:
            self.sensor = None
            self.get_logger().warn(f'BNO055 connect failed: {e}; will retry')
            return False

    def timer_callback(self):
        if self.sensor is None:
            # Retry connection at ~1 Hz until the sensor shows up.
            if time.monotonic() - self._last_connect_attempt >= 1.0:
                self.connect()
            return

        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Orientation. Adafruit returns the quaternion as (w, x, y, z);
            # map each component to its matching field (the original code put
            # w into .x, etc). The bus also silently corrupts ACKed reads
            # (~7% of transactions on the bench), so gate each field on a
            # physical-plausibility check and leave it zeroed when it fails:
            # downstream treats a zero quaternion/field as "not available".
            quat = self.sensor.quaternion
            if quat is not None and None not in quat:
                n2 = quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2
                if 0.5 <= n2 <= 1.5:  # unit quaternion, generous margin
                    msg.orientation.w = quat[0]
                    msg.orientation.x = quat[1]
                    msg.orientation.y = quat[2]
                    msg.orientation.z = quat[3]

            # Angular velocity (rad/s). BNO055 full scale is 2000 dps = 34.9.
            gyro = self.sensor.gyro
            if gyro is not None and None not in gyro:
                if gyro[0] ** 2 + gyro[1] ** 2 + gyro[2] ** 2 <= 35.0 ** 2:
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]

            # Linear acceleration, gravity removed (m/s^2). Anything past a
            # few g on a sub is a corrupted read, not motion.
            accel = self.sensor.linear_acceleration
            if accel is not None and None not in accel:
                if accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2 <= 30.0 ** 2:
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]

            # Gravity vector (m/s^2, body frame). Own try-block so a failed
            # gravity read doesn't cost the whole IMU sample, and a magnitude
            # gate: anything not ~9.81 m/s^2 long is a corrupted read, and a
            # corrupted vertical reference is worse than a stale one.
            try:
                grav = self.sensor.gravity
                if grav is not None and None not in grav:
                    g2 = grav[0] ** 2 + grav[1] ** 2 + grav[2] ** 2
                    if 8.0 ** 2 <= g2 <= 11.6 ** 2:
                        gmsg = Vector3Stamped()
                        gmsg.header = msg.header
                        gmsg.vector.x = grav[0]
                        gmsg.vector.y = grav[1]
                        gmsg.vector.z = grav[2]
                        self.gravity_publisher.publish(gmsg)
            except (OSError, ValueError, RuntimeError):
                pass

            self.publisher.publish(msg)
            self._consec_failures = 0
        except (OSError, ValueError, RuntimeError) as e:
            self._consec_failures += 1
            if self._consec_failures >= self._reconnect_after:
                self.get_logger().warn(
                    f'BNO055: {self._consec_failures} consecutive read failures ({e}); reconnecting')
                self._consec_failures = 0
                self.sensor = None
            else:
                self.get_logger().warn(f'BNO055 read failed: {e}',
                                       throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)

    node = ImuInput()

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
