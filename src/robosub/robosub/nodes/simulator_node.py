#!/usr/bin/env python3
import pygame
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from robosub.simulator.simulator import SubmarineSimulator
from robosub.sub.data_structures import ThrusterCommands

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

        self._image_pub    = self.create_publisher(Image,             '/camera/image_raw', 10)
        self._depth_pub    = self.create_publisher(Float32,           '/sensors/depth',    self.qos)
        self._heading_pub  = self.create_publisher(Float32,           '/sensors/heading',  self.qos)
        self._roll_pub     = self.create_publisher(Float32,           '/sensors/roll',     self.qos)
        self._imu_pub      = self.create_publisher(Imu,               '/sensors/imu',      self.qos)
        self._velocity_pub = self.create_publisher(Twist,             '/sensors/velocity', self.qos)
        self._ctrl_pub     = self.create_publisher(String,            '/sim/control',      10)

        self.create_subscription(Float32MultiArray, '/thruster_commands', self._thruster_cb, self.qos)
        self.create_subscription(String, '/sub/status',   self._status_cb, 10)
        self.create_subscription(String, '/sim/control',  self._ctrl_cb,   10)

        self.sim = SubmarineSimulator()

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

    def publish_sensors(self):
        p = self.sim.subPhysics
        imu = self.sim.last_imu_readings
        now = self.get_clock().now().to_msg()

        # Publish sensors with Best Effort QoS
        self._depth_pub.publish(Float32(data=float(p.z)))
        self._heading_pub.publish(Float32(data=float(p.heading)))
        self._roll_pub.publish(Float32(data=float(p.roll)))

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.angular_velocity.z = float(imu.gyro_z)
        imu_msg.angular_velocity.x = float(imu.gyro_x)
        imu_msg.linear_acceleration.x = float(imu.accel_x)
        imu_msg.linear_acceleration.y = float(imu.accel_y)
        imu_msg.linear_acceleration.z = float(imu.accel_z)
        self._imu_pub.publish(imu_msg)

        vel = Twist()
        vel.linear.x, vel.linear.y, vel.linear.z = float(p.velocity_x), float(p.velocity_y), float(p.velocity_z)
        self._velocity_pub.publish(vel)

        camera_np = np.ascontiguousarray(np.transpose(pygame.surfarray.array3d(self.sim.cameraSurface), (1, 0, 2))[:, :, ::-1])
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(camera_np, encoding='bgr8'))

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
                node.publish_sensors()
                sim.render()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__': main()