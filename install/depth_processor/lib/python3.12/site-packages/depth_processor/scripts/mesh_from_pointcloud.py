import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import Delaunay
import matplotlib.cm as cm

def plot_smoothed_vertices(vertices, isosurf=True, filename=None):
    # Convert to NumPy array
    vertices_np = np.array(vertices)

    # Create a figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Denoising the point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices_np)
    
    # Apply statistical outlier removal
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Convert back to NumPy
    vertices_np = np.asarray(pcd.points)
    x, y, z = vertices_np[:, 0], vertices_np[:, 1], vertices_np[:, 2]

    if isosurf:
        # Triangulate the surface using Delaunay
        tri = Delaunay(vertices_np[:, :2])
        ax.plot_trisurf(x, y, z, triangles=tri.simplices, cmap='viridis', alpha=0.8, linewidth=0.1)
    else:
        # Smooth scatter plot
        ax.scatter(x, y, z, c=z, cmap='viridis', marker='o', s=5, edgecolors='k', alpha=0.6)

    # Labels and aspect ratio
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1, 1, 0.5])
    ax.view_init(elev=25, azim=45)

    # Show or save the plot
    if filename:
        plt.savefig(filename, bbox_inches='tight')
    else:
        plt.show()



def create_colored_mesh(vertices, method="poisson", filename=None):
    # Convert to Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(vertices))

    # Denoise the cloud
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Estimate & fix normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=50))
    pcd.orient_normals_consistent_tangent_plane(100)

    if method == "poisson":
        # Poisson with lower depth
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=6)

        # Trim artifacts
        densities = np.asarray(densities)
        threshold = np.percentile(densities, 10)
        mesh = mesh.select_by_index(np.where(densities > threshold)[0])

    else:
        # BPA instead of Poisson
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 3 * avg_dist
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector([radius, radius * 1.5, radius * 2])
        )

    # Apply color mapping
    vertices_np = np.asarray(mesh.vertices)
    z_values = vertices_np[:, 2]
    z_min, z_max = z_values.min(), z_values.max()
    z_norm = (z_values - z_min) / (z_max - z_min)
    cmap = cm.get_cmap('terrain')
    colors = cmap(z_norm)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([mesh])

    if filename:
        o3d.io.write_triangle_mesh(filename, mesh)

    return mesh

def create_raw_mesh(vertices, alpha=0.02, filename=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(vertices))

    # Denoise to avoid floating artifacts
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)

    # Compute normals for visualization
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=50))
    
    # Alpha Shape (Delaunay-like triangulation)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

    # Apply height-based color mapping
    vertices_np = np.asarray(mesh.vertices)
    z_values = vertices_np[:, 2]
    z_min, z_max = z_values.min(), z_values.max()
    z_norm = (z_values - z_min) / (z_max - z_min)
    cmap = cm.get_cmap('terrain')
    colors = cmap(z_norm)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([mesh])

    if filename:
        o3d.io.write_triangle_mesh(filename, mesh)

    return mesh

if __name__ == "__main__":
    #generate_mesh()
    vertices = np.load("./pointclouds/global_pointcloud_2025-03-12_12-39-26.npy")
    #plot_smoothed_vertices(vertices, isosurf=True)
    # create_colored_mesh(vertices, method="bpa")
    create_raw_mesh(vertices, alpha=0.2)