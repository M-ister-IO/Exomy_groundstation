import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from threading import Thread
from argparse import ArgumentParser
from typing import Optional

from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

class Option:
    cfg: Optional[str]

class TOFPublisher(Node):
    def __init__(self, options: Option):
        super().__init__("tof_publisher")
        
        self.tof_ = self.__init_camera(options)
        if self.tof_ is None:
            raise Exception("Failed to initialize camera")

        self.pointsize_ = self.width_ * self.height_
        self.publisher_depth_ = self.create_publisher(Float32MultiArray, "tof_data", 10)
        self.running_ = True
        self.timer_ = self.create_timer(1 / 30, self.publish_depth_data)  # 30 Hz

    def __init_camera(self, options: Option):
        self.get_logger().info("Initializing ToF camera...")
        tof = ArducamCamera()
        ret = tof.open(Connection.CSI, 0) if options.cfg is None else tof.openWithFile(options.cfg, 0)
        
        if ret != 0:
            self.get_logger().error(f"Failed to open camera. Error code: {ret}")
            return None

        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            self.get_logger().error(f"Failed to start camera. Error code: {ret}")
            tof.close()
            return None
        
        info = tof.getCameraInfo()
        self.width_ = info.width
        self.height_ = info.height
        self.get_logger().info(f"Camera initialized. Resolution: {self.width_}x{self.height_}")
        return tof

    def publish_depth_data(self):
        frame = self.tof_.requestFrame(200)
        if frame is None or not isinstance(frame, DepthData):
            self.get_logger().warn("Failed to capture valid ToF depth frame!")
            return

        depth_buf = np.array(frame.depth_data, dtype=np.float32).reshape(self.height_, self.width_)
        self.tof_.releaseFrame(frame)

        # Publish the raw depth data as a flattened array
        depth_msg = Float32MultiArray()
        depth_msg.data = depth_buf.flatten().tolist()
        self.publisher_depth_.publish(depth_msg)

    def stop(self):
        self.running_ = False
        self.tof_.stop()
        self.tof_.close()


def main(args=None):
    rclpy.init(args=args)
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    ns = parser.parse_args()
    
    options = Option()
    options.cfg = ns.cfg
    
    tof_publisher = TOFPublisher(options)
    rclpy.spin(tof_publisher)
    
    tof_publisher.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
