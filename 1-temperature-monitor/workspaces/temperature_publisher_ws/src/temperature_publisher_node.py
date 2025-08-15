#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from temperature_monitoring_interfaces.msg import TemperatureData 
 
class TemperaturePublisherNode(Node): 
    def __init__(self):
        super().__init__("temperature_publisher_node")
        self.publisher_ = self.create_publisher(TemperatureData, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info("Publisher Node has been started")

    def publish_temperature(self):
        msg = TemperatureData()
        msg.temperature = 26.0 # Example temperature value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.temperature}"')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()