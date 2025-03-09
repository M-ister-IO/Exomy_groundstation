#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import ArducamDepthCamera as ac

class ToFImagePublisher(Node):
    def __init__(self):
        super().__init__('tof_image_publisher')

        # Create a publisher to publish image data to the "/tof_image" topic
        self.image_pub = self.create_publisher(Image, '/tof_image', 10)

        # Create a CvBridge object to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Initialize the Arducam camera
        self.cam = ac.ArducamCamera()
        ret = self.cam.open(ac.Connection.CSI, 0)
        if ret != 0:
            self.get_logger().error(f"Camera initialization failed with error code: {ret}")
            return

        ret = self.cam.start(ac.FrameType.RAW)
        if ret != 0:
            self.get_logger().error(f"Failed to start camera with error code: {ret}")
            self.cam.close()
            return

        self.get_logger().info("Arducam camera started successfully.")

        # Create a timer to call the publish_tof_image function at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_tof_image)

    def publish_tof_image(self):
        # Request a frame from the camera
        frame = self.cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.RawData):
            buf = frame.raw_data
            self.cam.releaseFrame(frame)

            # Process the image buffer (e.g., normalize the data)
            buf = (buf / (1 << 4)).astype(np.uint8)  # Example processing to scale the values

            # Convert the OpenCV image (numpy array) to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(buf, encoding="mono8")

            # Publish the image to the "/tof_image" topic
            self.image_pub.publish(ros_image)

    def stop_camera(self):
        # Stop the camera and clean up
        self.cam.stop()
        self.cam.close()

def main(args=None):
    rclpy.init(args=args)

    tof_image_publisher = ToFImagePublisher()

    try:
        rclpy.spin(tof_image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tof_image_publisher.stop_camera()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
