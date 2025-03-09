#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String

def processed_data_callback(data):
    # Check the processed IMU data and send a command based on some criteria
    acceleration_magnitude = data.data
    command = ""

    # Example criteria: If acceleration is greater than 10, send 'MOVE' command
    if acceleration_magnitude > 10:
        command = "MOVE"
    else:
        command = "STOP"
    
    rospy.loginfo(f"Sending command: {command}")

    # Publish the command to the /command topic
    command_pub.publish(command)

def brain_listener():
    # Initialize the ROS node
    rospy.init_node('brain_listener', anonymous=True)

    # Subscribe to the processed IMU data
    rospy.Subscriber("/processed_imu_data", Float64, processed_data_callback)

    # Create a publisher to send commands to the /command topic
    global command_pub
    command_pub = rospy.Publisher("/command", String, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    brain_listener()
