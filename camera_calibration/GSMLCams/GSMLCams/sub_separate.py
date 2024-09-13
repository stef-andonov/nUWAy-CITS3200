import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# extends node acting as ROS2 subscriber for images
class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__('camera_image_subscriber')

        # Create a subscription to the image topic, listening on the '/video_frames' topic
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',  # Replace with image topic
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()        # Initialize the CvBridge object to convert ROS Image messages to OpenCV images

        self.image_counter = 0          # Counter keeping track of the number of saved images

    
    # Callback function that gets triggered when an image message is received
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        image_filename = f'calibration_image_{self.image_counter}.jpg'        # Create a filename for the saved image, using the image counter to ensure unique filenames
        os.makedirs("pictures", exist_ok=True)                                # Ensure that the 'pictures' directory exists, create it if it doesn't
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
