import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/video_feed', 10)
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device at index 0.")
            rclpy.shutdown()
        self.bridge = CvBridge()
        self.get_logger().info("Camera Publisher node started, streaming from system camera.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Could not convert frame to Image message: {e}')
        else:
            self.get_logger().warn('Could not read frame from camera.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

