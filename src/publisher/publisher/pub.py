#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePub(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.freq_ = 1
        self.counter = 0
        self.get_logger().info("Publishing in frequency %d" % self.freq_)

        self.timer_ = self.create_timer(self.freq_, self.timerCallBack)
    
    def timerCallBack(self):
        msg = String()
        msg.data = "Hello ROS " + str(self.counter)
        self.pub_.publish(msg)
        self.counter += 1

def main():
    rclpy.init()
    simple_publisher = SimplePub()
    try:
            rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
            print("Terminating Node...")
            simple_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__== '__main__':
    main()