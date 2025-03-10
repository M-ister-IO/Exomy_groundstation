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

                

            # Transform points to the correct position using /tf
            try:
                transform = self.tf_buffer.lookup_transform('world', 'imu_link', rclpy.time.Time())
                translation = np.array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])
                transformed_points = []
                for point in points:
                    point_stamped = PointStamped()
                    point_stamped.header.frame_id = 'world'
                    point_stamped.point.x = point[0]
                    point_stamped.point.y = point[1]
                    point_stamped.point.z = point[2]
                    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                    transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
                points = np.array(transformed_points)
                # Apply rotation and translation
                points = points + translation
            except tf2_ros.LookupException:
                self.get_logger().warn("Transform not found, skipping transformation")


            # Remove points that are too close to each other in the new frame (using KDTree)
            if len(points) > 0:
                tree = cKDTree(points)  # Create KD-Tree from points
                unique_points = []
                visited = set()

                for i, point in enumerate(points):
                    
                    if i in visited or i%10!=0:
                        continue  # Skip if already removed

                    unique_points.append(point)

                    # Find all nearby points within 0.01 meters
                    self.get_logger().info(f"Processing point {i}/{len(points)}")
                    indices = tree.query_ball_point(point, 0.03)
                    visited.update(indices)  # Mark them as visited

                points = np.array(unique_points)

            # Filter points based on distance to existing points in the global map
            if self.points:
                tree = cKDTree(self.points)
                distances, _ = tree.query(points, distance_upper_bound=0.03)
                new_points = points[distances > 0.03]
            else:
                new_points = points



            # Add new points to the list
            self.points.extend(new_points)

            # Create and publish PointCloud2 message
            pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, self.points)
            self.get_logger().info("Point cloud published")
            self.publisher_.publish(pc2_msg_)


def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = DepthProcessor()
    rclpy.spin(point_cloud_processor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
