#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

def tof_image_callback(data):
    # Process the ToF image (this could be depth or other information)
    bridge = CvBridge()
    try:
        # Convert the ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Get image dimensions
        height, width = cv_image.shape

        # Extract the center pixel depth value
        center_x = width // 2
        center_y = height // 2
        center_depth = float(cv_image[center_y, center_x])  # Convert to float

        rospy.loginfo(f"Distance at image center: {center_depth} mm")

        # Publish the processed data (e.g., center pixel distance)
        processed_image_pub.publish(center_depth)

    except Exception as e:
        rospy.logerr(f"Error processing ToF image: {e}")

def listener():
    # Initialize the ROS node
    rospy.init_node('tof_data_processor', anonymous=True)

    # Subscribe to the raw ToF image topic
    rospy.Subscriber("/tof_image", Image, tof_image_callback)

    # Create publisher to publish the processed ToF image data
    global processed_image_pub
    processed_image_pub = rospy.Publisher("/processed_tof_image", Float64, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
