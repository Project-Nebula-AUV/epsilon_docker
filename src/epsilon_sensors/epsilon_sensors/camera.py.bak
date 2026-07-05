import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class CameraInput(Node):
    def __init__(self):
        super().__init__('camera_input')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')

        # --- Color / white-balance controls -------------------------------
        # The OV9782 leaves a warm (red-heavy) cast even with auto white
        # balance on, which can push true whites out of the white HSV range
        # and skew the red/green thresholds. These let the colors be made
        # neutral without editing code.
        #   auto_wb            : enable the camera's hardware auto white balance.
        #   wb_temperature     : >=0 forces manual WB (disables auto) at this
        #                        Kelvin value (e.g. 5500). -1 = leave to auto_wb.
        #   saturation/gain/brightness/contrast/sharpness : >=0 sets that
        #                        V4L2 control; -1 = leave the camera default.
        #   gray_world         : software white balance -- per-frame, scales B/G/R
        #                        so their means match, removing any global tint.
        #                        Robust on the bench; underwater it can amplify
        #                        red-channel noise, so tune against real footage.
        self.declare_parameter('auto_wb', True)
        self.declare_parameter('wb_temperature', -1)
        self.declare_parameter('saturation', -1)
        self.declare_parameter('gain', -1)
        self.declare_parameter('brightness', -1)
        self.declare_parameter('contrast', -1)
        self.declare_parameter('sharpness', -1)
        self.declare_parameter('gray_world', False)

        device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gray_world = bool(self.get_parameter('gray_world').value)

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # OV9782 streams MJPG; request it so higher resolutions/fps are available.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._apply_color_controls()
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device}')

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(
            f'Publishing /camera/image_raw at {self.width}x{self.height} bgr8 '
            f'from device {device} (gray_world={self.gray_world})')

    def _apply_color_controls(self):
        """Push the white-balance / color params to the V4L2 device. A cap.set
        for an unsupported control is a harmless no-op."""
        wb_temp = self.get_parameter('wb_temperature').value
        if wb_temp is not None and wb_temp >= 0:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(wb_temp))
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_WB,
                         1.0 if bool(self.get_parameter('auto_wb').value) else 0.0)
        for name, prop in (('saturation', cv2.CAP_PROP_SATURATION),
                           ('gain', cv2.CAP_PROP_GAIN),
                           ('brightness', cv2.CAP_PROP_BRIGHTNESS),
                           ('contrast', cv2.CAP_PROP_CONTRAST),
                           ('sharpness', cv2.CAP_PROP_SHARPNESS)):
            val = self.get_parameter(name).value
            if val is not None and val >= 0:
                self.cap.set(prop, float(val))

    @staticmethod
    def _gray_world_balance(frame):
        """Neutralize a global color cast: scale each channel so the channel
        means converge to the overall gray mean."""
        f = frame.astype(np.float32)
        means = f.reshape(-1, 3).mean(axis=0) + 1e-6
        gray = float(means.mean())
        f *= (gray / means)
        return np.clip(f, 0, 255).astype(np.uint8)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        if self.gray_world:
            frame = self._gray_world_balance(frame)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
