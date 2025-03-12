import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import TransformStamped, PointStamped
import numpy as np
from scipy.spatial import cKDTree
import tf2_ros
import tf2_geometry_msgs
import struct
from datetime import datetime
from matplotlib import pyplot as plt

class DepthProcessor(Node):
    def __init__(self):
        super().__init__("depth_processor")
        self.points = []
        self.depth_sub = self.create_subscription(Float32MultiArray, "raw_depth", self.depth_callback, 1)
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.header = Header()
        self.header.frame_id = "world"
        self.width_ = 240-60
        self.height_ = 180-60
        self.fx = 192.92
        self.fy = 191.25

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("DepthProcessor initialized")
        self.get_logger().info("Subscriptions and publisher created")

    def depth_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Depth data received")
        self.depth_data = np.array(msg.data) / 1000  # Convert to meters
        self.process_point_cloud()

    def process_point_cloud(self):
        self.get_logger().info("Processing point cloud")

        if hasattr(self, 'depth_data'):
            z = self.depth_data
            z[z <= 0] = np.nan  # Handle invalid depth values
            z[z > 2.1] = np.nan  # Handle invalid depth values
            z = z.reshape((self.height_, self.width_))  # Ensure correct shape

            # Generate u and v coordinate grids
            u = np.arange(self.width_)
            v = np.arange(self.height_)
            u, v = np.meshgrid(u, v)

            # Compute x and y coordinates
            x = (u - self.width_ / 2) * z / self.fx
            y = (v - self.height_ / 2) * z / self.fy

            # Stack into (x, y, z) point format
            points = np.stack((x, y, z), axis=-1)
            points = points[~np.isnan(points).any(axis=-1)]  # Remove invalid points

            # Rotation matrix for 45° counterclockwise around X-axis
            R_x = np.array([[1,  0,  0], 
                            [0,  np.cos(np.radians(-90-45)), -np.sin(np.radians(-90-45))], 
                            [0,  np.sin(np.radians(-90-45)),  np.cos(np.radians(-90-45))]])

            # Apply rotation
            points = (R_x @ points.T).T  # Rotate points
            points = points + [0,0,0.355]

                

            # Transform points to the correct position using /tf
            try:
                transform = self.tf_buffer.lookup_transform('world', 'imu_link', rclpy.time.Time())
                translation = np.array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])
                self.get_logger().info(f"{translation}")
                transformed_points = []
                for point in points:
                    point_stamped = PointStamped()
                    point_stamped.header.frame_id = 'world'
                    point_stamped.point.x = point[0]
                    point_stamped.point.y = point[1]
                    point_stamped.point.z = point[2]
                    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                    transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
                self.get_logger().info(f"New frame before tf p1: {points[0]}")
                points = np.array(transformed_points)
                # Apply rotation and translation
                
                self.get_logger().info(f"New frame after tf p1: {points[0]}")
            except tf2_ros.LookupException:
                self.get_logger().warn("Transform not found, skipping transformation")


            # Remove points that are too close to each other in the new frame (using KDTree)
            if len(points) > 0:
                tree = cKDTree(points)  # Create KD-Tree from points
                unique_points = []
                visited = set()

                for i, point in enumerate(points):
                    
                    if i in visited : #or i%2==0:
                        continue  # Skip if already removed

                    unique_points.append(point)

                    # Find all nearby points within 0.01 meters
                    #self.get_logger().info(f"Processing point {i}/{len(points)}")
                    indices = tree.query_ball_point(point, 0.02)
                    visited.update(indices)  # Mark them as visited

                points = np.array(unique_points)

            # Filter points based on distance to existing points in the global map
            if self.points:
                tree = cKDTree(self.points)
                distances, _ = tree.query(points, distance_upper_bound=0.02)
                new_points = points[distances > 0.02]
            else:
                new_points = points



            # Add new points to the list
            self.points.extend(new_points)

            # Create and publish PointCloud2 message
            self.pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, self.points)
            self.get_logger().info("Point cloud published")
            self.publisher_.publish(self.pc2_msg_)

    def save_pcd(self):
        self.get_logger().info("Received PointCloud2 message")

        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename_npy = f"/home/charles/ros2_ws/src/depth_processor/depth_processor/scripts/pointclouds/pointcloud_{timestamp}.npy"

        # Convert PointCloud2 to numpy array
        cloud_array = self.pointcloud2_to_xyz(self.pc2_msg_)

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
    point_cloud_processor = DepthProcessor()
    try:
        rclpy.spin(point_cloud_processor)
    except:
        point_cloud_processor.get_logger().info("Ctrl+C detected, saving PointCloud2...")
        point_cloud_processor.save_pcd()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
