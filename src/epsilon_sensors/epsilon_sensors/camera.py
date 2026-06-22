import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2


class CameraInput(Node):
    def __init__(self):
        super().__init__('camera_input')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')

        device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # OV9782 streams MJPG; request it so higher resolutions/fps are available.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device}')

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(
            f'Publishing /camera/image_raw at {self.width}x{self.height} bgr8 '
            f'from device {device}')

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

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
