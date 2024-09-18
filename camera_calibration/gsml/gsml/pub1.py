# Based on sample code from KieranQB ROS2 Repository
# rclpy - Python API for interacting with ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Extends node to act as a ROS2 publisher for video frames
class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'video_frames_1', 10)
        timer_period = 1 / 60    # 60 fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #video capture object to capture frames from the default camera (device 0)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        # Load calibration results - cam matrix, distortion coefficients
        self.camera_matrix = np.array([])
        self.dist_coeffs = np.array([])
        self.load_calibration_data('camera_calibration_results.txt')

    def load_calibration_data(self, filename):
        try:
            # open and read calibration file
            with open(filename, 'r') as f:
                lines = f.readlines()

                # Extract the 3x3 camera matrix from file
                self.camera_matrix = np.array([list(map(float, line.split())) for line in lines[1:4]], dtype=np.float32).reshape((3, 3))
                
                # Extract distortion coefficients
                self.dist_coeffs = np.array(list(map(float, lines[6].split())), dtype=np.float32)

                # succesful calibration data loading
                self.get_logger().info("Calibration data loaded successfully.")
                self.get_logger().info(f"Camera Matrix: \n{self.camera_matrix}")
                self.get_logger().info(f"Distortion Coefficients: \n{self.dist_coeffs}")
        except Exception as e:
            # error - calibration file cant be loaded
            self.get_logger().error(f"Failed to load calibration data: {e}")

    def timer_callback(self):
        # capture frame from camera
        ret, frame = self.cap.read()

        if ret:
            # If loaded, undistort the frame, using https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
            if self.camera_matrix.size > 0 and self.dist_coeffs.size > 0:
                h, w = frame.shape[:2]
                # Calculate the optimal new camera matrix and ROI
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 0, (w, h))
                # Undistort the captured frame
                undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

                # Optionally crop the image based on the ROI (Region Of Interest)
                x, y, w, h = roi
                undistorted_frame = undistorted_frame[y:y+h, x:x+w]

                # Convert the undistorted frame to ROS Image message
                ros_image = self.br.cv2_to_imgmsg(undistorted_frame)
                # Publish ROS image message
                self.publisher.publish(ros_image)
                self.get_logger().info("Publishing Undistorted Video Frame")
            else:
                # No calibration data
                ros_image = self.br.cv2_to_imgmsg(frame)
                self.publisher.publish(ros_image)
                self.get_logger().warn("No calibration data available. Publishing raw video frame")
        else:
            self.get_logger().error("Failed to get frame data from camera: {e}")

def main(args=None):
    rclpy.init(args=args)

    image_pub = ImagePublisher()        # create instance in image publisher

    rclpy.spin(image_pub)        # keep node running and listening

    # clean up before shutdown
    image_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
