#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from temperature_monitoring_interfaces.msg import TemperatureData 
import random
import math
 
class TemperaturePublisherNode(Node): 
    def __init__(self):
        super().__init__("temperature_publisher_node")

        self.declare_parameter("publishing_frequency", 1.0)
        self.declare_parameter("temperature_range", [20.0, 30.0])
        self.declare_parameter("sensor_id", "sensor_1")

        self.frequency = self.get_parameter("publishing_frequency").value
        self.get_logger().info(f"Publishing frequency set to {self.frequency} seconds")

        self.temperature_range = self.get_parameter("temperature_range").value
        self.get_logger().info(f"Temperature range set to {self.temperature_range}")

        self.sensor_id = self.get_parameter("sensor_id").value
        self.get_logger().info(f"Sensor ID set to {self.sensor_id}")

        self.publisher_ = self.create_publisher(TemperatureData, 'temperature', 1)
        self.timer = self.create_timer(self.frequency, self.publish_temperature)
        self.get_logger().info("Publisher Node has been started")

        self.phase = 0.0

    def publish_temperature(self):
        msg = TemperatureData()
        # Simulate a sine wave between temperature_range[0] and temperature_range[1]
        amplitude = (self.temperature_range[1] - self.temperature_range[0]) / 2
        offset = (self.temperature_range[1] + self.temperature_range[0]) / 2
        msg.temperature = offset + amplitude * math.sin(self.phase)
        self.phase += 0.1  # Adjust step for desired frequency
        msg.sensor_id = self.sensor_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.temperature}", Sensor ID: "{self.sensor_id}"')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()