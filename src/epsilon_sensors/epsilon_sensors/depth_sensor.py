import rclpy
from epsilon_sensors import ms5837
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import time
import threading


class DepthInput(Node):
    def __init__(self):
        super().__init__('depth_node')

        rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self.period = 1.0 / float(rate_hz)

        # DENSITY_FRESHWATER (997) or DENSITY_SALTWATER (1029)
        self.fluid_density = self.declare_parameter(
            'fluid_density', ms5837.DENSITY_FRESHWATER).value

        self.publisher = self.create_publisher(Float32, '/depth', 10)

        self.sensor = None
        self._lock = threading.Lock()
        self._running = True

        # Attempt initial connection
        self.connect()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('MS5837 depth node started')

    def connect(self):
        try:
            sensor = ms5837.MS5837_30BA(bus=2)
            if not sensor.init():
                raise RuntimeError('MS5837 init() returned False')
            sensor.setFluidDensity(self.fluid_density)
            time.sleep(0.1)
            
            with self._lock:
                self.sensor = sensor
            self.get_logger().info('MS5837 connected')
            return True
        except (OSError, ValueError, RuntimeError) as e:
            with self._lock:
                self.sensor = None
            self.get_logger().warn(f'MS5837 connect failed: {e}; will retry')
            return False

    def _read_loop(self):
        while self._running:
            with self._lock:
                sensor = self.sensor

            if sensor is None:
                self.connect()
                time.sleep(1.0)
                continue

            try:
                # OSR_256 or OSR_8192
                if not sensor.read(ms5837.OSR_8192):
                    self.get_logger().warn('MS5837 read() returned False')
                    time.sleep(self.period)
                    continue

                msg = Float32()
                msg.data = sensor.depth()
                self.publisher.publish(msg)

            except (OSError, ValueError, RuntimeError) as e:
                self.get_logger().warn(f'MS5837 read failed: {e}; reconnecting')
                with self._lock:
                    self.sensor = None

            time.sleep(self.period)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DepthInput()

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