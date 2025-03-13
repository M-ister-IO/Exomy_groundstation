import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from scipy.spatial import cKDTree
from datetime import datetime
import struct
from matplotlib import pyplot as plt

class PointCloudCreator(Node):
    def __init__(self):
        super().__init__("pointcloud_creator1")
        self.points = []
        self.depth_sub = self.create_subscription(Float32MultiArray, "raw_depth2", self.depth_callback, 1)
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud2", 10)
        self.header = Header()
        self.header.frame_id = "world"
        self.width_ = 240-100
        self.height_ = 180-100
        self.fx = 192.92
        self.fy = 191.25
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("PointCloudCreator1 initialized")

    def depth_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Depth data received")
        depth_data = np.array(msg.data) / 1000  # Convert to meters
        self.process_point_cloud(depth_data)

    def process_point_cloud(self, depth_data):
        z = depth_data.reshape((self.height_, self.width_))
        z[z <= 0] = np.nan
        z[z > 2.1] = np.nan
        
        u, v = np.meshgrid(np.arange(self.width_), np.arange(self.height_))
        x = (u - self.width_ / 2) * z / self.fx
        y = (v - self.height_ / 2) * z / self.fy
        points = np.stack((x, y, z), axis=-1)
        points = points[~np.isnan(points).any(axis=-1)]
        R_x = np.array([[1,  0,  0], 
                            [0,  np.cos(np.radians(-90-45)), -np.sin(np.radians(-90-45))], 
                            [0,  np.sin(np.radians(-90-45)),  np.cos(np.radians(-90-45))]])

        # Apply rotation
        points = (R_x @ points.T).T  # Rotate points
        points = points + [1.2,-0.3,0.28]

        transformed_points = self.transform_points(points)
        unique_points = self.filter_points(transformed_points)
        self.points.extend(unique_points)
        
        self.pc2_msg = point_cloud2.create_cloud_xyz32(self.header, self.points)
        self.publisher_.publish(self.pc2_msg)
        self.get_logger().info("Point cloud published")

    def transform_points(self, points):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'world', rclpy.time.Time())
            transformed_points = []
            for point in points:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'world'
                point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = point
                transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
            return np.array(transformed_points)
        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not found, skipping transformation")
            return points

    def filter_points(self, points):
        if not self.points:
            return points.tolist()
        try:
            tree = cKDTree(self.points)
            distances, _ = tree.query(points, distance_upper_bound=0.01)
            return points[distances > 0.01].tolist()
        except:
            return points.tolist()

    def save_pcd(self):
        self.get_logger().info("Received PointCloud2 message")

        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename_npy = f"/home/claudio/Exomy_groundstation/src/depth_processor/depth_processor/scripts/pointclouds/pointcloud2_{timestamp}.npy"

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
    node = PointCloudCreator()
    try:
        rclpy.spin(node)
    except:
        node.get_logger().info("Ctrl+C detected, saving PointCloud2...")
        node.save_pcd()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
