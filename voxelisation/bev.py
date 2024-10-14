import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

#Load PCD data from a file
def load_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

#Convert PCD to voxel representation
def create_voxel_grid(pcd, voxel_size=0.1):
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    return voxel_grid

#Generate Bird's-Eye View (BEV) of voxel grid
def get_bev(voxel_grid):
    voxels = np.asarray(voxel_grid.get_voxels())
    voxel_centers = np.asarray([voxel.grid_index for voxel in voxels])
    bev = voxel_centers[:, [0, 1]]  # Project to XY plane
    return bev

#Plot BEV with bounding box
def plot_bev(bev):
    plt.scatter(bev[:, 0], bev[:, 1], marker='s', s=1)
    
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("BEV")
    plt.show()

#Visualize point cloud and bounding box
def visualize_pcd_voxels(pcd):
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # File path to PCD file
    pcd_file = "data/1718334181F.pcd"
    
    # Load the PCD file
    pcd = load_pcd(pcd_file)
    
    # Create a voxel grid from the point cloud
    voxel_grid = create_voxel_grid(pcd, voxel_size=0.1)
    
    # Get Bird's-Eye View (BEV) projection
    bev = get_bev(voxel_grid)
    
    # Plot BEV
    plot_bev(bev)
    
    # Visualize the point cloud in 3d
    visualize_pcd_voxels(pcd)
