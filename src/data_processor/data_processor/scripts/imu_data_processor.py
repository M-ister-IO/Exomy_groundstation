#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
import numpy as np

def imu_callback(data):
    # Process IMU data (accelerometer and gyroscope)
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z
    gx = data.angular_velocity.x
    gy = data.angular_velocity.y
    gz = data.angular_velocity.z

    # Example processing: Calculate the magnitude of acceleration
    acceleration_magnitude = np.sqrt(ax**2 + ay**2 + az**2)

    rospy.loginfo(f"Processed Acceleration Magnitude: {acceleration_magnitude}")

    # Publish the processed data (e.g., acceleration magnitude)
    processed_data_pub.publish(acceleration_magnitude)

def magnetic_field_callback(data):
    # Process magnetic field data (magnetometer)
    mx = data.magnetic_field.x
    my = data.magnetic_field.y
    mz = data.magnetic_field.z

    # Example processing: Calculate the magnitude of the magnetic field
    magnetic_field_magnitude = np.sqrt(mx**2 + my**2 + mz**2)

    rospy.loginfo(f"Processed Magnetic Field Magnitude: {magnetic_field_magnitude}")

    # You can publish the processed magnetic field data if needed
    # processed_mag_data_pub.publish(magnetic_field_magnitude)

def listener():
    # Initialize the ROS node
    rospy.init_node('imu_data_processor', anonymous=True)

    # Subscribe to the raw IMU data topic
    rospy.Subscriber("/imu_data", Imu, imu_callback)

    # Subscribe to the raw magnetic field data topic
    rospy.Subscriber("/magnetic_field_data", MagneticField, magnetic_field_callback)

    # Create publisher to publish the processed IMU data
    global processed_data_pub
    processed_data_pub = rospy.Publisher("/processed_imu_data", Float64, queue_size=10)

    # If you want to publish the processed magnetic field data, you can create a publisher for it
    global processed_mag_data_pub
    processed_mag_data_pub = rospy.Publisher("/processed_mag_data", Float64, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
