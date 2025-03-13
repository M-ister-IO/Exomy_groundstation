import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
from scipy.spatial import cKDTree
from datetime import datetime
import struct
from matplotlib import pyplot as plt

class GlobalPointCloudMapper(Node):
    def __init__(self):
        super().__init__("global_mapping_node")
        self.global_points = []
        self.pointcloud1_sub = self.create_subscription(PointCloud2, "point_cloud1", self.pointcloud1_callback, 10)
        self.pointcloud2_sub = self.create_subscription(PointCloud2, "point_cloud2", self.pointcloud2_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, "pointcloud_global", 10)
        self.header = Header()
        self.header.frame_id = "world"
        self.get_logger().info("GlobalPointCloudMapper initialized")

    def pointcloud1_callback(self, msg: PointCloud2):
        self.get_logger().info("Received point cloud 1")
        self.update_global_map(msg)

    def pointcloud2_callback(self, msg: PointCloud2):
        self.get_logger().info("Received point cloud 2")
        self.update_global_map(msg)

    def update_global_map(self, msg: PointCloud2):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    
        # Convert the points into a NumPy array with float64 type
        new_points = np.array([(p['x'], p['y'], p['z']) for p in points], dtype=np.float64)

        if len(self.global_points) == 0:
            self.global_points = new_points.tolist()
        else:
            try: 
                tree = cKDTree(self.global_points)
                distances, _ = tree.query(new_points, distance_upper_bound=0.03)
                unique_points = new_points[distances > 0.03].tolist()
                self.global_points.extend(unique_points)
            except:
                pass
            
        
        self.pc2_msg = point_cloud2.create_cloud_xyz32(self.header, self.global_points)
        self.publisher_.publish(self.pc2_msg)
        self.get_logger().info("Global point cloud updated and published")

    def save_pcd(self):
        self.get_logger().info("Received PointCloud2 message")

        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename_npy = f"/home/claudio/Exomy_groundstation/src/depth_processor/depth_processor/scripts/pointclouds/global_pointcloud_{timestamp}.npy"

        # Convert PointCloud2 to numpy array
        cloud_array = self.pointcloud2_to_xyz(self.pc2_msg)

        # Save as .npy for fast loading
        np.save(filename_npy, cloud_array)
        self.get_logger().info(f"Saved pointcloud as {filename_npy}")
        self.plot_vertices(cloud_array,isosurf=True)
        self.plot_vertices2(cloud_array,isosurf=True)


    def pointcloud2_to_xyz(self, cloud_msg):
        """Convert sensor_msgs/PointCloud2 to a Numpy array."""
        assert isinstance(cloud_msg, PointCloud2)
        fields = cloud_msg.fields
        data = cloud_msg.data
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step

        # Find X, Y, Z offsets
        offset_map = {field.name: field.offset for field in fields}
        x_offset, y_offset, z_offset = offset_map['x'], offset_map['y'], offset_map['z']

        # Extract points
        num_points = len(data) // point_step
        cloud_array = np.zeros((num_points, 3), dtype=np.float32)

        for i in range(num_points):
            point_data = data[i * point_step: (i + 1) * point_step]
            x = struct.unpack_from('f', point_data, x_offset)[0]
            y = struct.unpack_from('f', point_data, y_offset)[0]
            z = struct.unpack_from('f', point_data, z_offset)[0]
            cloud_array[i] = [x, y, z]

        return cloud_array

    def plot_vertices(self,vertices, isosurf = False, filename = None):
        # Create a new plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x = [v[0] for v in vertices]
        y = [v[1] for v in vertices]
        z = [v[2] for v in vertices]
        if isosurf:
            ax.plot_trisurf(x, y, z, linewidth=0.2, antialiased=True)
        else:
            ax.scatter(x, y, z, c='r', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # Show or save the plot
        plt.axis('scaled')
        if filename is None:
            plt.show()
        else:
            plt.savefig(filename)

    def plot_vertices2(self,vertices, isosurf=False, filename=None):
        # Create a new plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x = [v[0] for v in vertices]
        y = [v[1] for v in vertices]
        z = [v[2] for v in vertices]

        # Use numpy array for smooth processing
        vertices_np = np.array(vertices)

        # Optional: Smoothing by fitting a surface
        if isosurf:
            ax.plot_trisurf(vertices_np[:, 0], vertices_np[:, 1], vertices_np[:, 2],
                            cmap='viridis', linewidth=0.2, antialiased=True, alpha=0.7)
        else:
            # Scatter plot with color map for better smoothness
            ax.scatter(x, y, z, c=z, cmap='viridis', marker='o', s=10, edgecolors='k', alpha=0.7)

        # Labels and other plot settings
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=20, azim=30)  # Adjusting the viewing angle for better perspective
        
        # Set aspect ratio and grid
        ax.set_box_aspect([1, 1, 1])  # Equal scaling for all axes
        ax.grid(True, linestyle='--', alpha=0.5)

        # Ensure that the axes have equal scale
        max_range = np.ptp(np.array([x, y, z])).max()  # Find the maximum range across all axes
        mid_x = np.mean(x)
        mid_y = np.mean(y)
        mid_z = np.mean(z)
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Show or save the plot
        if filename is None:
            plt.show()
        else:
            plt.savefig(filename, bbox_inches='tight')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPointCloudMapper()

    try:
        rclpy.spin(node)
    except:
        node.get_logger().info("Ctrl+C detected, saving GlobalPointCloud...")
        node.save_pcd()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
