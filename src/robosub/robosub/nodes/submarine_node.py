#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

from robosub.sub.submarine import Submarine
from robosub.sub.data_structures import SensorSuite, MPU6050Readings
from robosub.mission import create_mission

class SubmarineNode(Node):
    def __init__(self):
        super().__init__('submarine_node')

        self._bridge = CvBridge()
        self._sub = Submarine(mission_plan=create_mission())
        self._paused  = False
        self._started = False   # waits for 'start' command before running mission

        # Define Best Effort QoS for low-latency control
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Sensors
        self._camera_image = None
        self._depth = 0.0
        self._heading = 0.0
        self._roll = 0.0
        self._imu = MPU6050Readings()
        self._vel_x, self._vel_y, self._vel_z = 0.0, 0.0, 0.0

        # Subscriptions using Best Effort QoS
        self.create_subscription(Image,   '/camera/image_raw', self._image_cb, 10) # Images usually stay reliable/large depth
        self.create_subscription(Float32, '/sensors/depth',    self._depth_cb,   self.qos)
        self.create_subscription(Float32, '/sensors/heading',  self._heading_cb, self.qos)
        self.create_subscription(Float32, '/sensors/roll',     self._roll_cb,    self.qos)
        self.create_subscription(Imu,     '/sensors/imu',      self._imu_cb,     self.qos)
        self.create_subscription(Twist,   '/sensors/velocity', self._vel_cb,     self.qos)
        self.create_subscription(String,  '/sim/control',      self._ctrl_cb,    10)

        self._cmd_pub    = self.create_publisher(Float32MultiArray, '/thruster_commands', self.qos)
        self._status_pub = self.create_publisher(String,            '/sub/status',        10)

        self._last_stamp = self.get_clock().now()
        self.create_timer(1.0 / 60.0, self._control_loop)

    def _image_cb(self, msg: Image):
        self._camera_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Float32): self._depth = float(msg.data)
    def _heading_cb(self, msg: Float32): self._heading = float(msg.data)
    def _roll_cb(self, msg: Float32): self._roll = float(msg.data)

    def _imu_cb(self, msg: Imu):
        self._imu = MPU6050Readings(
            accel_x=float(msg.linear_acceleration.x),
            accel_y=float(msg.linear_acceleration.y),
            accel_z=float(msg.linear_acceleration.z),
            gyro_x=float(msg.angular_velocity.x),
            gyro_z=float(msg.angular_velocity.z),
        )

    def _vel_cb(self, msg: Twist):
        self._vel_x, self._vel_y, self._vel_z = float(msg.linear.x), float(msg.linear.y), float(msg.linear.z)

    def _ctrl_cb(self, msg: String):
        if msg.data == 'start':
            self._started = True
            self._paused  = False
        elif msg.data == 'reset':
            self._sub.reset()
            self._started = False
            self._paused  = False
        elif msg.data == 'pause':  self._paused = True
        elif msg.data == 'resume': self._paused = False
        elif msg.data == 'quit':   rclpy.try_shutdown()

    def _control_loop(self):
        if not self._started:
            status = String(data='WAITING|Press Start to begin')
            self._status_pub.publish(status)
            return
        if self._paused or self._camera_image is None: return
        now = self.get_clock().now()
        dt = (now - self._last_stamp).nanoseconds / 1e9
        self._last_stamp = now
        if dt <= 0.0 or dt > 0.1: return

        sensors = SensorSuite(
            camera_image=self._camera_image, depth=self._depth,
            heading=self._heading, roll=self._roll, imu=self._imu,
            velocity_x=self._vel_x, velocity_y=self._vel_y, velocity_z=self._vel_z
        )

        commands, _ = self._sub.update(dt, sensors)

        out = Float32MultiArray()
        out.data = [float(commands.hfl), float(commands.hfr), float(commands.hal),
                    float(commands.har), float(commands.vp), float(commands.vs)]
        self._cmd_pub.publish(out)

        status = String(data=f"{self._sub.get_current_task_name()}|{self._sub.get_current_state_name()}")
        self._status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = SubmarineNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__': main()