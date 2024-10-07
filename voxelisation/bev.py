import open3d as o3d
import numpy as np
import cv2

# Load point cloud from PCD file
pcd = o3d.io.read_point_cloud("data/1718334764R.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd])

# Convert point cloud to numpy array (X, Y, Z)
point_cloud = np.asarray(pcd.points)

# Extract X and Y coordinates for BEV
xy_points = point_cloud[:, :2]

# Calculate BEV bounding box: min/max X and Y
x_min, y_min = np.min(xy_points, axis=0)
x_max, y_max = np.max(xy_points, axis=0)

# Create an empty image for visualization
bev_image_size = 500
bev_image = np.zeros((bev_image_size, bev_image_size, 3), dtype=np.uint8)

# Scale factor for converting real-world coordinates to pixel coordinates
scaling_factor = 100

# Convert the bounding box points to image coordinates (from BEV plane)
x_min_pixel = int((x_min + 10) * scaling_factor)
x_max_pixel = int((x_max + 10) * scaling_factor)
y_min_pixel = int((y_min + 10) * scaling_factor)
y_max_pixel = int((y_max + 10) * scaling_factor)

# Draw the bounding box on the BEV image
cv2.rectangle(bev_image, 
              (x_min_pixel, y_min_pixel), 
              (x_max_pixel, y_max_pixel), 
              color=(0, 255, 0), thickness=2)

# Display the result
cv2.imshow("BEV Bounding Box", bev_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
