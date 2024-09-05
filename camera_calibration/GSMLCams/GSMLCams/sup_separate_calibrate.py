import cv2
import numpy as np
import glob

def calibrate_camera(image_folder, chessboard_size, square_size_mm):
    # Prepare object points based on the chessboard size
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    # Scale object points to the actual size of the squares
    objp *= square_size_mm / 1000.0  # Convert from mm to meters

    objpoints = []
    imgpoints = []

    # Load images from the folder
    images = glob.glob(f'{image_folder}/*.jpg')

    if not images:
        print("No images found in the specified directory.")
        return

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    if len(objpoints) == 0 or len(imgpoints) == 0:
        print("Chessboard corners not found in any images.")
        return

    # Perform calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not ret:
        print("Camera calibration failed.")
        return

    # Save the calibration results to a text file
    with open('camera_calibration_results.txt', 'w') as f:
        f.write("Camera Matrix:\n")
        np.savetxt(f, camera_matrix, fmt='%.6f')
        f.write("\nDistortion Coefficients:\n")
        np.savetxt(f, dist_coeffs, fmt='%.6f')

    print("Camera calibration complete.")
    print("Results saved to 'camera_calibration_results.txt'.")

if __name__ == '__main__':
    calibrate_camera('pictures', (13, 12), 12)  # Adjust path and square size as needed
