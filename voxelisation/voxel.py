import open3d as o3d

import os

# Directory containing the point cloud files
data_directory = "data"

# Loop through the files in the directory
for filename in os.listdir(data_directory):
    if filename.endswith(".pcd"):
        file_path = os.path.join(data_directory, filename)
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(file_path)

        # Set the voxel size
        voxel_size = 0.5

        # Voxelize the point cloud
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

        # Visualize the voxelized point cloud
        o3d.visualization.draw_geometries([voxel_grid])

        # Optionally, save the voxelized point cloud to a file
        voxel_grid_file_path = os.path.join(data_directory, f"voxelized_{filename}")
        o3d.io.write_voxel_grid(voxel_grid_file_path, voxel_grid)
