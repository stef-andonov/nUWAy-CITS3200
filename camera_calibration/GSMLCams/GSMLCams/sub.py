import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolov7_package import Yolov7Detector

class ImageSub(Node):

    def __init__(self):
        super().__init__('Image_sub')

        self.subscriber = self.create_subscription(Image, 'video_frames', self.listener_callback, 1)

        self.subscriber

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info("Image recieved!")

        current_frame = self.br.imgmsg_to_cv2(msg)

        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSub()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__== '__main__':
    main()