import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu

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

        self.sensor = None
        self._last_connect_attempt = 0.0

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
            # w into .x, etc).
            quat = self.sensor.quaternion
            if quat is not None and None not in quat:
                msg.orientation.w = quat[0]
                msg.orientation.x = quat[1]
                msg.orientation.y = quat[2]
                msg.orientation.z = quat[3]

            # Angular velocity (rad/s).
            gyro = self.sensor.gyro
            if gyro is not None and None not in gyro:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]

            # Linear acceleration, gravity removed (m/s^2).
            accel = self.sensor.linear_acceleration
            if accel is not None and None not in accel:
                msg.linear_acceleration.x = accel[0]
                msg.linear_acceleration.y = accel[1]
                msg.linear_acceleration.z = accel[2]

            self.publisher.publish(msg)
        except (OSError, ValueError, RuntimeError) as e:
            # Lost the sensor (glitch / unplug). Drop it and let the timer
            # reconnect instead of crashing the node.
            self.get_logger().warn(f'BNO055 read failed: {e}; reconnecting')
            self.sensor = None


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