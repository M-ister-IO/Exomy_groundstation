#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from time import sleep

class JoyPublisher(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('joy_publisher')
        
        # Create publisher for /joy topic
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)

        # Initialize joystick axes and buttons state
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example joystick axes data
        self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # Example joystick buttons data

    def change_mode(self, mode):
        if mode == '1':
            self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif mode == '2':
            self.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif mode == '3':
            self.buttons = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # NOT WORKING idk what number crabbing is have to figure out listening to joystick in website when publishing in /joy

        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def go_front(self):
        self.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def turn_left(self):
        self.axes = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def turn_right(self):
        self.axes = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def go_back(self):
        self.axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def stop(self):
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Stop
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.publish_message()

    def publish_message(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.publisher_.publish(msg)

    def square_up(self, square_size, turn_time, refresh_time):
        for i in range(4):
            # Change mode to Ackermann
            self.change_mode('1')
            sleep(refresh_time)
            self.go_front()
            sleep(square_size)

            # Change mode to Spot Turning
            self.change_mode('2')
            sleep(refresh_time)
            # turn right for 1 seconds
            self.turn_right()
            sleep(turn_time)

    def line_up(self, line_size, turn_time, refresh_time):
        for i in range(1):
            # Change mode to Ackermann
            self.change_mode('1')
            sleep(refresh_time)
            self.go_front()
            sleep(line_size)

    def spin_out(self, turn_time, direction, refresh_time):
        # Change mode to Spot Turning
        self.change_mode('2')
        sleep(refresh_time)
        # turn right for 1 seconds
        if direction == 1:
            self.turn_left()
        else:
            self.turn_right()
        sleep(turn_time)

    def start(self):
        # Start with Ackermann mode
        self.change_mode('1')
        sleep(0.5)  # wait for change mode to be done

        # Sequence of movements
        while rclpy.ok():
            # Perform sequence
            self.line_up(3, 2.2, 0.5)
            self.spin_out(1, 1, 0.5)
            self.line_up(3, 2.2, 0.5)
            self.spin_out(1, -1, 0.5)

            # Stop after the sequence
            self.change_mode('1')
            self.stop()
            break


def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    joy_publisher.start()  # Start the movement sequence
    rclpy.spin(joy_publisher)  # Keep the node running

    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
