#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
 
class TemperaturePublisherNode(Node): 
    def __init__(self):
        super().__init__("temperature_publisher_node")
        self.publisher_ = self.create_publisher(String, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info("Publisher Node has been started")

    def publish_temperature(self):
        msg = String()
        msg.data = "25.0" # Example temperature value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()