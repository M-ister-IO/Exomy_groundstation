import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class PathPlanningPublisher(Node):
    def __init__(self):
        super().__init__('path_planning_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'path_planning', 10)
        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        data_2d = [
            [1.0, 2.0],
            [4.0, 5.0]
        ]
        flattened_data = [item for sublist in data_2d for item in sublist]
        
        msg = Float32MultiArray()
        msg.data = flattened_data

        # Define layout dimensions.
        rows = len(data_2d)
        cols = len(data_2d[0]) if rows > 0 else 0

        # For the first dimension (rows)
        dim0 = MultiArrayDimension()
        dim0.label = "rows"
        dim0.size = rows
        # The stride for the first dimension is total number of elements.
        dim0.stride = rows * cols

        # For the second dimension (columns)
        dim1 = MultiArrayDimension()
        dim1.label = "columns"
        dim1.size = cols
        # The stride for the inner dimension is 1 times the number of columns.
        dim1.stride = cols

        msg.layout.dim.append(dim0)
        msg.layout.dim.append(dim1)
        msg.layout.data_offset = 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing 2D array: {data_2d}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
