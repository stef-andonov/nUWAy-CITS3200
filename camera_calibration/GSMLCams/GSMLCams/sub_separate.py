import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__('camera_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',  # Replace with your image topic
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_counter = 0

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_filename = f'calibration_image_{self.image_counter}.jpg'
        # Ensure the directory exists
        os.makedirs("pictures", exist_ok=True)
        image_path = os.path.join("pictures", image_filename)
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'Saved {image_filename}')
        self.image_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CameraImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
