from launch import LaunchDescription
from launch_ros.actions import Node


# This launch file starts the temperature publisher and subscriber nodes
# with specified parameters for the temperature monitoring application.
# It sets the publishing frequency, temperature range, and sensor ID for the publisher node.
# The subscriber node listens to the temperature data published by the publisher node.
# The nodes are configured to output their logs to the screen for easy monitoring.

def generate_launch_description():
    ld = LaunchDescription()

    temperature_publisher =  Node(
            package='temperature_publisher',
            executable='temperature_publisher_node',
            parameters=[
                {'publishing_frequency': 1.0},
                {'temperature_range': [20.0, 30.0]},
                {'sensor_id': 'Robot Temp Sensor'}
            ]
        )
     
    temperature_subscriber = Node(
            package='temperature_subscriber',
            executable='temperature_subscriber_node',

        )

    ld.add_action(temperature_publisher)
    ld.add_action(temperature_subscriber)


    return ld