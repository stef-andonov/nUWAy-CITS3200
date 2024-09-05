import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSub(Node):

    def __init__(self):
        super().__init__('Image_sub')

        self.subscriber = self.create_subscription(Image, 'video_frames', self.listener_callback, 1)

        self.br = CvBridge()
        self.calibration_images = []
        self.is_calibrated = False
        self.calibrate_on = False  # Toggle this to start calibration

        # Calibration parameters
        self.pattern_size = (13, 12)  # Chessboard pattern size (number of inner corners per row and column)
        self.square_size = 0.012  # Size of a square in meters (adjust to your actual chessboard square size)

    def listener_callback(self, msg):
        self.get_logger().info("Image received!")

        current_frame = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

        if self.calibrate_on:
            self.calibration_images.append(current_frame)
            if len(self.calibration_images) >= 20:  # Adjust as needed
                self.perform_calibration()
                self.calibrate_on = False

    def perform_calibration(self):
        if len(self.calibration_images) == 0:
            self.get_logger().error("No calibration images collected")
            return

        # Prepare object points and image points
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.square_size

        objpoints = []
        imgpoints = []

        for img in self.calibration_images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                self.get_logger().info("Chessboard corners found and added to points")

        if len(objpoints) > 0 and len(imgpoints) > 0:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            if ret:
                self.get_logger().info("Calibration successful")
                np.savez('camera_calibration.npz', mtx=mtx, dist=dist)
            else:
                self.get_logger().error("Calibration failed")
        else:
            self.get_logger().error("Not enough points for calibration")

def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSub()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
