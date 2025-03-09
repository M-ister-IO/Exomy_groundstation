#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

def command_callback(data):
    # Check the command and publish to the /joy topic if criteria are met
    command = data.data
    joy_msg = Joy()

    if command == "MOVE":
        # Example: Set the joystick values to move (you can change these values as per your requirements)
        joy_msg.axes = [1.0, 0.0, 0.0, 0.0]  # Set axes for movement
        joy_msg.buttons = [1, 0, 0, 0]  # Example button press for MOVE command
        rospy.loginfo(f"Publishing to /joy: MOVE")
    elif command == "STOP":
        # Example: Stop the movement (set all axes to zero)
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0]
        rospy.loginfo(f"Publishing to /joy: STOP")
    else:
        # Default values if the command is unknown
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0]

    # Publish to the /joy topic
    joy_pub.publish(joy_msg)

def command_listener():
    # Initialize the ROS node
    rospy.init_node('command_listener', anonymous=True)

    # Subscribe to the /command topic
    rospy.Subscriber("/command", String, command_callback)

    # Create a publisher to send joystick messages to the /joy topic
    global joy_pub
    joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    command_listener()
