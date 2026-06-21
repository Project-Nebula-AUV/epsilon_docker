#!/usr/bin/env python3
"""
Recorder node — subscribes to /camera/image_raw and writes a timestamped
MP4 debug video. Overlays task/state, thruster HUD, and frame counter.

Topics subscribed:
  /camera/image_raw      sensor_msgs/Image
  /sub/status            std_msgs/String           ("TaskName|state_name")
  /thruster_commands     std_msgs/Float32MultiArray ([hfl, hfr, hal, har, vp, vs])
  /sim/control           std_msgs/String           ("quit" closes file cleanly)
"""
import os
import math
import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32MultiArray, Float32
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge


class RecorderNode(Node):

    def __init__(self):
        super().__init__('recorder_node')

        self.declare_parameter('output_dir', os.path.expanduser('~/robosub_recordings'))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('scale', 2.0)

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._fps   = self.get_parameter('fps').get_parameter_value().double_value
        self._scale = self.get_parameter('scale').get_parameter_value().double_value

        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = os.path.join(output_dir, f'run_{timestamp}.mp4')

        self._bridge       = CvBridge()
        self._writer       = None
        self._out_path     = output_path
        self._task_name    = ''
        self._state_name   = ''
        self._thrusters    = [0.0] * 6   # hfl, hfr, hal, har, vp, vs
        self._depth        = 0.0
        self._heading      = 0.0
        self._roll         = 0.0
        self._gyro_z       = 0.0         # yaw rate rad/s
        self._frame_count  = 0
        self._started      = False       # don't record until 'start' received
        self._sensor_count = 0           # counts heading/pitch/imu callbacks received

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Image,            '/camera/image_raw',   self._image_cb,    10)
        self.create_subscription(String,           '/sub/status',         self._status_cb,   10)
        self.create_subscription(Float32MultiArray,'/thruster_commands',  self._thruster_cb, qos)
        self.create_subscription(Float32,          '/sensors/depth',      self._depth_cb,    qos)
        self.create_subscription(Float32,          '/sensors/heading',    self._heading_cb,  qos)
        self.create_subscription(Float32,          '/sensors/roll',       self._roll_cb,     qos)
        self.create_subscription(Imu,              '/sensors/imu',        self._imu_cb,      qos)
        self.create_subscription(String,           '/sim/control',        self._ctrl_cb,     10)

        self.get_logger().info(f'Recording to {output_path}')

    # --- Callbacks ---

    def _status_cb(self, msg: String):
        parts = msg.data.split('|', 1)
        self._task_name  = parts[0]
        self._state_name = parts[1] if len(parts) > 1 else ''

    def _thruster_cb(self, msg: Float32MultiArray):
        if len(msg.data) == 6:
            self._thrusters = list(msg.data)

    def _depth_cb(self, msg: Float32):
        self._depth = float(msg.data)

    def _heading_cb(self, msg: Float32):
        self._heading = float(msg.data)
        self._sensor_count += 1

    def _roll_cb(self, msg: Float32):
        self._roll = float(msg.data)

    def _imu_cb(self, msg: Imu):
        self._gyro_z = float(msg.angular_velocity.z)

    def _ctrl_cb(self, msg: String):
        if msg.data == 'start':
            self._started = True
        elif msg.data == 'reset':
            self._close()
            # Open a fresh file for the new run
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = os.path.dirname(self._out_path)
            self._out_path = os.path.join(output_dir, f'run_{timestamp}.mp4')
            self._frame_count = 0
            self._started = False
        elif msg.data == 'quit':
            self._close()
            rclpy.try_shutdown()

    def _image_cb(self, msg: Image):
        if not self._started:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self._scale != 1.0:
            h, w = frame.shape[:2]
            frame = cv2.resize(frame, (int(w * self._scale), int(h * self._scale)),
                               interpolation=cv2.INTER_NEAREST)

        if self._writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._writer = cv2.VideoWriter(self._out_path, fourcc, self._fps, (w, h))
            self.get_logger().info(f'VideoWriter opened: {w}x{h} @ {self._fps} fps')

        self._draw_overlay(frame)
        self._frame_count += 1
        self._writer.write(frame)

    # --- Drawing ---

    def _put(self, frame, text, x, y, scale=0.45, color=(255, 255, 255)):
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, text, (x+1, y+1), font, scale, (0,0,0), 1, cv2.LINE_AA)
        cv2.putText(frame, text, (x,   y  ), font, scale, color,   1, cv2.LINE_AA)

    def _draw_thruster_bar(self, frame, x, y, label, value):
        """Draw a vertical bar: green for positive, red for negative."""
        bar_w, bar_h = 16, 50
        cy = y + bar_h // 2

        # Background outline
        cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (100, 100, 100), 1)

        # Centre line
        cv2.line(frame, (x, cy), (x + bar_w, cy), (180, 180, 180), 1)

        # Filled bar
        if value > 0:
            top = int(cy - value * bar_h / 2)
            cv2.rectangle(frame, (x+1, top), (x + bar_w - 1, cy), (0, 200, 0), -1)
        elif value < 0:
            bot = int(cy + abs(value) * bar_h / 2)
            cv2.rectangle(frame, (x+1, cy), (x + bar_w - 1, bot), (0, 0, 200), -1)

        # Label below
        font  = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.3
        (tw, _), _ = cv2.getTextSize(label, font, scale, 1)
        lx = x + bar_w // 2 - tw // 2
        cv2.putText(frame, label, (lx+1, y + bar_h + 10), font, scale, (0,0,0),       1, cv2.LINE_AA)
        cv2.putText(frame, label, (lx,   y + bar_h +  9), font, scale, (220,220,220), 1, cv2.LINE_AA)

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        m = 6

        # Task name (top-left, larger)
        if self._task_name:
            self._put(frame, self._task_name, m, m + 14, scale=0.5, color=(255, 255, 100))

        # State name (below task)
        if self._state_name:
            self._put(frame, self._state_name, m, m + 30, scale=0.38, color=(200, 200, 200))

        # Sensor data (below state)
        sensor_lines = [
            f'Depth:   {self._depth:.2f} m',
            f'Heading: {self._heading:.2f} deg ({self._sensor_count})',
            f'Roll:    {self._roll:.2f} deg',
            f'YawRate: {math.degrees(self._gyro_z):.2f} deg/s',
        ]
        for i, line in enumerate(sensor_lines):
            self._put(frame, line, m, m + 48 + i * 14, scale=0.35, color=(180, 220, 255))

        # Frame counter (top-right)
        self._put(frame, f'#{self._frame_count}', w - 55, m + 14, scale=0.38)

        # Thruster HUD (bottom of frame)
        labels  = ['HFL', 'HFR', 'HAL', 'HAR', 'VP',  'VS']
        spacing = 24
        n       = len(labels)
        total_w = n * spacing
        hud_x   = (w - total_w) // 2
        hud_y   = h - 75

        # "Thrusters" label centred above bars
        self._put(frame, 'Thrusters', hud_x, hud_y - 6, scale=0.35, color=(200, 200, 200))

        for i, (label, value) in enumerate(zip(labels, self._thrusters)):
            self._draw_thruster_bar(frame, hud_x + i * spacing, hud_y, label, value)

    # --- Lifecycle ---

    def _close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self.get_logger().info(
                f'Recording saved: {self._out_path} ({self._frame_count} frames)')

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
