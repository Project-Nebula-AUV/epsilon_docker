"""
camera_input.py — Arducam OV9782 ROS2 camera node
===================================================

Publishes
---------
  /camera/image_raw   sensor_msgs/Image   bgr8 frames

Subscribes
----------
  /camera/controls    std_msgs/String     JSON object with any subset of
                                          the control parameters; applied
                                          immediately on receipt.

Parameters (set via --params-file or ros2 param set)
----------
  device          int     Camera index (default 0)
  width           int     Frame width  (default 320)
  height          int     Frame height (default 240)
  fps             float   Publish rate (default 30.0)
  frame_id        str     TF frame id  (default 'camera')

  # White balance
  auto_wb         bool    Hardware auto white balance (default True)
  wb_temperature  int     Manual WB in Kelvin; >=0 disables auto_wb (default -1)

  # V4L2 controls  (-1 = leave camera default)
  saturation      int     (default -1)
  gain            int     (default -1)
  brightness      int     (default -1)
  contrast        int     (default -1)
  sharpness       int     (default -1)

  # Exposure
  auto_exposure   bool    Hardware auto exposure (default True)
  exposure        int     Manual exposure in 100 µs units; >=0 disables
                          auto_exposure (default -1)

  # Software
  gray_world      bool    Per-frame gray-world white-balance (default False)

Resilience
----------
  The device is opened with retries at startup and is automatically
  released + reopened from inside the frame loop if it fails to open or
  stops returning frames (transient USB drop / device-busy). One bad open
  no longer kills the node for the whole run — important because the nav
  brain emits nothing until it receives a camera frame.

YAML params file example (pass with --params-file camera_params.yaml)
----------------------------------------------------------------------
  camera_input:
    ros__parameters:
      device: 0
      width: 1280
      height: 720
      fps: 30.0
      auto_wb: true
      wb_temperature: -1
      auto_exposure: true
      exposure: -1
      saturation: -1
      gain: -1
      brightness: -1
      contrast: -1
      sharpness: -1
      gray_world: false

Runtime control via topic
-------------------------
  ros2 topic pub --once /camera/controls std_msgs/String \
      'data: "{\"exposure\": 150, \"auto_exposure\": false}"'
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ── Control descriptor ────────────────────────────────────────────────────────

class ControlDef:
    """Metadata for a single camera control."""
    __slots__ = ("param", "prop", "is_bool", "default")

    def __init__(self, param: str, prop, is_bool: bool = False, default=None):
        self.param   = param    # ROS2 parameter name
        self.prop    = prop     # cv2.CAP_PROP_* constant (None for software-only)
        self.is_bool = is_bool  # treat as on/off flag
        self.default = default  # default value


# All hardware V4L2 controls we manage, in declaration order.
V4L2_CONTROLS: list[ControlDef] = [
    ControlDef("saturation",  cv2.CAP_PROP_SATURATION,  default=-1),
    ControlDef("gain",        cv2.CAP_PROP_GAIN,         default=-1),
    ControlDef("brightness",  cv2.CAP_PROP_BRIGHTNESS,   default=-1),
    ControlDef("contrast",    cv2.CAP_PROP_CONTRAST,     default=-1),
    ControlDef("sharpness",   cv2.CAP_PROP_SHARPNESS,    default=-1),
]


# ── Node ─────────────────────────────────────────────────────────────────────

class CameraInput(Node):

    def __init__(self):
        super().__init__("camera_input")

        # ── Declare all parameters ────────────────────────────────────────
        self.declare_parameter("device",   0)
        self.declare_parameter("width",    320)
        self.declare_parameter("height",   240)
        self.declare_parameter("fps",      30.0)
        self.declare_parameter("frame_id", "camera")

        # White balance
        self.declare_parameter("auto_wb",        True)
        self.declare_parameter("wb_temperature", -1)

        # Exposure
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure",      -1)

        # V4L2 scalar controls
        for cd in V4L2_CONTROLS:
            self.declare_parameter(cd.param, cd.default)

        # Software
        self.declare_parameter("gray_world", False)

        # ── Read startup values ───────────────────────────────────────────
        self.device   = self.get_parameter("device").value
        self.width    = self.get_parameter("width").value
        self.height   = self.get_parameter("height").value
        fps           = self.get_parameter("fps").value
        self.frame_id = self.get_parameter("frame_id").value

        # ── Open camera (resilient) ───────────────────────────────────────
        # The capture is opened with a few retries here, and the frame loop
        # reopens it if it ever drops, so a transient open/USB failure no
        # longer leaves the nav brain frame-starved for the whole run.
        self.cap = None
        self._consec_fail = 0
        # Attempt a reopen at most ~once per second while frames are failing.
        self._reopen_interval = max(1, int(round(fps)))

        for attempt in range(5):
            if self._open():
                break
            self.get_logger().error(
                f"Failed to open camera device {self.device} "
                f"(attempt {attempt + 1}/5); retrying…")
            time.sleep(0.5)
        else:
            self.get_logger().error(
                f"Camera device {self.device} not open after retries; "
                f"the frame loop will keep retrying.")

        # ── ROS2 pub / sub / timer ────────────────────────────────────────
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        self.controls_sub = self.create_subscription(
            String,
            "/camera/controls",
            self._on_controls_msg,
            10,
        )

        self.timer = self.create_timer(1.0 / fps, self._timer_callback)

        self.get_logger().info(
            f"camera_input ready — {self.width}x{self.height} @ {fps} fps  "
            f"device={self.device}  opened={self._is_open()}  "
            f"gray_world={self.get_parameter('gray_world').value}"
        )

    # ── Capture open / reopen ───────────────────────────────────────────────────

    def _is_open(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def _open(self) -> bool:
        """(Re)open the capture device. Returns True if it is open afterwards."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap = cap

        if cap.isOpened():
            self._apply_all_controls()
            return True
        return False

    def _handle_capture_failure(self) -> None:
        """Count a failed/absent frame and reopen the device periodically."""
        self._consec_fail += 1
        self.get_logger().warn("Frame capture failed", throttle_duration_sec=5.0)
        if self._consec_fail % self._reopen_interval == 0:
            self.get_logger().warn(
                f"Reopening camera device {self.device} after "
                f"{self._consec_fail} consecutive failed frames",
                throttle_duration_sec=5.0)
            if self._open():
                self.get_logger().info(
                    f"Camera device {self.device} reopened successfully")

    # ── Control application ───────────────────────────────────────────────────

    def _apply_all_controls(self) -> None:
        """Push every control parameter to the V4L2 driver."""
        self._apply_wb()
        self._apply_exposure()
        for cd in V4L2_CONTROLS:
            self._apply_scalar(cd.param, cd.prop)

    def _apply_wb(self) -> None:
        wb_temp = self.get_parameter("wb_temperature").value
        if wb_temp is not None and wb_temp >= 0:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(wb_temp))
            self.get_logger().info(f"WB: manual @ {wb_temp} K")
        else:
            auto = bool(self.get_parameter("auto_wb").value)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1.0 if auto else 0.0)
            self.get_logger().info(f"WB: {'auto' if auto else 'off'}")

    def _apply_exposure(self) -> None:
        exposure = self.get_parameter("exposure").value
        if exposure is not None and exposure >= 0:
            # V4L2: auto_exposure=1 → manual, auto_exposure=3 → auto
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
            self.get_logger().info(f"Exposure: manual @ {exposure}")
        else:
            auto = bool(self.get_parameter("auto_exposure").value)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0 if auto else 1.0)
            self.get_logger().info(f"Exposure: {'auto' if auto else 'off'}")

    def _apply_scalar(self, param_name: str, prop) -> None:
        val = self.get_parameter(param_name).value
        if val is not None and val >= 0:
            self.cap.set(prop, float(val))

    # ── /camera/controls subscriber ───────────────────────────────────────────

    def _on_controls_msg(self, msg: String) -> None:
        """
        Accept a JSON object whose keys are parameter names.
        Any recognised key is applied immediately to the driver and also
        written back to the ROS2 parameter server so `ros2 param get` stays
        in sync.

        Example payload:
            {"exposure": 150, "auto_exposure": false, "saturation": 80}
        """
        try:
            updates: dict = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"/camera/controls: bad JSON — {exc}")
            return

        if not isinstance(updates, dict):
            self.get_logger().warn("/camera/controls: expected a JSON object")
            return

        changed_wb       = False
        changed_exposure = False
        scalar_params    = {cd.param for cd in V4L2_CONTROLS}

        for key, value in updates.items():
            # Validate the parameter exists
            try:
                self.get_parameter(key)
            except Exception:
                self.get_logger().warn(f"/camera/controls: unknown parameter '{key}' — skipped")
                continue

            # Write back to the parameter server
            param = rclpy.parameter.Parameter(key, value=value)
            self.set_parameters([param])

            if key in ("auto_wb", "wb_temperature"):
                changed_wb = True
            elif key in ("auto_exposure", "exposure"):
                changed_exposure = True
            elif key == "gray_world":
                pass  # software-only, picked up per-frame
            elif key in scalar_params:
                cd = next(c for c in V4L2_CONTROLS if c.param == key)
                self._apply_scalar(key, cd.prop)

        if changed_wb:
            self._apply_wb()
        if changed_exposure:
            self._apply_exposure()

        self.get_logger().info(f"Controls updated: {list(updates.keys())}")

    # ── Frame loop ────────────────────────────────────────────────────────────

    @staticmethod
    def _gray_world_balance(frame: np.ndarray) -> np.ndarray:
        """Scale B/G/R channels so their means converge to the overall grey mean."""
        f     = frame.astype(np.float32)
        means = f.reshape(-1, 3).mean(axis=0) + 1e-6
        gray  = float(means.mean())
        f    *= gray / means
        return np.clip(f, 0, 255).astype(np.uint8)

    def _timer_callback(self) -> None:
        if not self._is_open():
            self._handle_capture_failure()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self._handle_capture_failure()
            return

        # A good frame: clear the failure streak.
        self._consec_fail = 0

        frame = cv2.rotate(frame, cv2.ROTATE_180)

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        if bool(self.get_parameter("gray_world").value):
            frame = self._gray_world_balance(frame)

        msg              = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height       = self.height
        msg.width        = self.width
        msg.encoding     = "bgr8"
        msg.is_bigendian = 0
        msg.step         = self.width * 3
        msg.data         = frame.tobytes()
        self.pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
