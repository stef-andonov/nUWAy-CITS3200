import cv2
import numpy as np
import glob
import argparse

def calibrate_camera(image_folder, chessboard_size, square_size_mm):
    # Prepare object points based on the chessboard size
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    # Scale object points to the actual size of the squares
    objp *= square_size_mm / 1000.0  # Convert from mm to meters

    # store object points and image points across mutliple aimges
    objpoints = []            #3D points in real-world image
    imgpoints = []            #2D points in image plane

    # Load images from the folder
    images = glob.glob(f'{image_folder}/*.jpg')

    if not images:
        print("No images found in the specified directory.")
        return

    # Loop through each image to find board corners
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            # If corners found, add object and image points
            objpoints.append(objp)
            imgpoints.append(corners)

            #Draw and display corners
            cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)            #Display for 500ms

    cv2.destroyAllWindows()

    # Ensure corners were found in atleast 1 image
    if len(objpoints) == 0 or len(imgpoints) == 0:
        print("Chessboard corners not found in any images.")
        return

    # Perform calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not ret:
        
        print("Camera calibration failed.")
        return

    # Save the calibration results - cam matrix and distortion coefficients - to a text file
    with open('camera_calibration_results.txt', 'w') as f:
        f.write("Camera Matrix:\n")
        np.savetxt(f, camera_matrix, fmt='%.6f')
        f.write("\nDistortion Coefficients:\n")
        np.savetxt(f, dist_coeffs, fmt='%.6f')

    print("Camera calibration complete.")
    print("Results saved to 'camera_calibration_results.txt'.")

def main(args=None):
    parser = argparse.ArgumentParser(description='Camera calibration script.')
    parser.add_argument('image_folder', type=str, help='Path to the folder containing calibration images.')
    parser.add_argument('chessboard_width', type=int, help='Width of the chessboard (number of inner corners per row).')
    parser.add_argument('chessboard_height', type=int, help='Height of the chessboard (number of inner corners per column).')
    parser.add_argument('square_size_mm', type=float, help='Size of the squares on the chessboard in millimeters.')
    
    args = parser.parse_args()

    calibrate_camera(args.image_folder, (args.chessboard_width, args.chessboard_height), args.square_size_mm)

if __name__ == '__main__':
    main()
