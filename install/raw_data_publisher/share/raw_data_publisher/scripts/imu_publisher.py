#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import random  # For generating random data (replace with real sensor data)

class ImuPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # Create publishers
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/magnetic_field_data', 10)

        # Timer to call the publish function at 10Hz
        self.timer = self.create_timer(0.1, self.publish_data)  # 0.1 seconds = 10Hz

        # Create message objects
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

    def publish_data(self):
        # Get the current time
        current_time = self.get_clock().now()

        # IMU message
        self.imu_msg.header.stamp = current_time.to_msg()
        self.imu_msg.header.frame_id = 'base_link'

        # Orientation (in quaternion)
        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = 0.0
        self.imu_msg.orientation.w = 1.0

        # Angular velocity (gyroscope data)
        self.imu_msg.angular_velocity.x = random.uniform(-10, 10)  # example values
        self.imu_msg.angular_velocity.y = random.uniform(-10, 10)
        self.imu_msg.angular_velocity.z = random.uniform(-10, 10)

        # Linear acceleration (accelerometer data)
        self.imu_msg.linear_acceleration.x = 0.0
        self.imu_msg.linear_acceleration.y = 0.0
        self.imu_msg.linear_acceleration.z = 9.81  # gravity (m/s^2)

        # Publish IMU message with accelerometer and gyroscope data
        self.imu_pub.publish(self.imu_msg)

        # Magnetic field (magnetometer data)
        self.mag_msg.magnetic_field.x = random.uniform(-50, 50)  # example values in microteslas (µT)
        self.mag_msg.magnetic_field.y = random.uniform(-50, 50)
        self.mag_msg.magnetic_field.z = random.uniform(-50, 50)

        # Publish magnetic field data separately
        self.mag_pub.publish(self.mag_msg)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher_node = ImuPublisherNode()

    rclpy.spin(imu_publisher_node)

    # Destroy the node explicitly (optional - done automatically when exiting)
    imu_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
