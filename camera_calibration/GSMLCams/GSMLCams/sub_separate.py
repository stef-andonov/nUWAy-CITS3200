import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# extends node acting as ROS2 subscriber for images
class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__('camera_image_subscriber')        #initialise nose with name

        # Create a subscription to the image topic, listening on the '/video_frames' topic
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',  # image topic name
            self.listener_callback,
            10        # Queue size for storing images
        )

        self.bridge = CvBridge()        # Initialize the CvBridge object to convert ROS Image messages to OpenCV images

        self.image_counter = 0          # Counter keeping track of the number of saved images

    
    # Callback function that gets triggered when an image message is received
    def listener_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image using CvBridge
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        # Generate a unique numbered filename for the saved image
        image_filename = f'calibration_image_{self.image_counter}.jpg'     
        # Ensure that the 'pictures' directory exists, create it if it doesn't
        os.makedirs("pictures", exist_ok=True)
        image_path = os.path.join("pictures", image_filename)
        # Save the OpenCV image to the specified path
        cv2.imwrite(image_path, cv_image)
        # Log the saved image's filename
        self.get_logger().info(f'Saved {image_filename}')
        # move on to next image
        self.image_counter += 1

def main(args=None):
    # Initialize ROS2 Python library
    rclpy.init(args=args)
    node = CameraImageSubscriber()
    # Spin node to keep it alive/responsive to incoming messages
    rclpy.spin(node)
    # clean up/shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
