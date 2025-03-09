import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import qwiic_icm20948
from std_msgs.msg import Float32MultiArray
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish every 100 ms

        # Initialize the IMU sensor
        self.IMU = qwiic_icm20948.QwiicIcm20948()

        # Subscribe to position data
        self.position_subscriber = self.create_subscription(
            Float32MultiArray,
            '/exomy/position',  # Replace with the correct topic name
            self.position_callback,
            10)

        # Check if the IMU is connected
        if not self.IMU.connected:
            self.get_logger().error("The Qwiic ICM20948 device isn't connected.")
            return

        # Initialize the IMU
        self.IMU.begin()

    def publish_imu_data(self):
        if self.IMU.dataReady():
            self.IMU.getAgmt()  # Read all axis and temp from sensor

            # Create an Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            imu_msg.header.frame_id = 'base_link'

            # Raw accelerometer data
            imu_msg.linear_acceleration.x = float('{: 06d}'.format(self.IMU.axRaw))
            imu_msg.linear_acceleration.y = float('{: 06d}'.format(self.IMU.ayRaw))
            imu_msg.linear_acceleration.z = float('{: 06d}'.format(self.IMU.azRaw))

            # Raw gyroscope data (not scaled)
            imu_msg.angular_velocity.x = float('{: 06d}'.format(self.IMU.gxRaw))
            imu_msg.angular_velocity.y = float('{: 06d}'.format(self.IMU.gyRaw))
            imu_msg.angular_velocity.z = float('{: 06d}'.format(self.IMU.gzRaw))

            # Passing magnetometer data in the unused linear_acceleration fields
            #imu_msg.linear_acceleration.x = self.IMU.mxRaw  # Magnetometer X
            #imu_msg.linear_acceleration.y = self.IMU.myRaw  # Magnetometer Y
            #imu_msg.linear_acceleration.z = self.IMU.mzRaw  # Magnetometer Z

            # Publish the raw IMU data
            self.imu_pub.publish(imu_msg)

        else:
            self.get_logger().info("Waiting for IMU data...")

    def shutdown(self):
        self.get_logger().info("Shutting down IMU publisher.")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
