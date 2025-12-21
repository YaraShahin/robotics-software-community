# Temperature Monitoring

## How to setup

1. `git clone `: 
    clone the monorepo. The repo already contains both workspaces for the publisher and subscriber containers.
2. `just setup-project`:
    a. symlinks the relevant packages from packages dir to each workspace src
    b. runs docker compose build to build both containers
    c. runs each container and installs all dependencies and builds the workspace.

## How to run

1. First Terminal:
    a. `just run-publisher`: will open the publisher container shell
    b. `cd `
    c. Optional: `colcon build --symlink-install && source install/setup.bash`: If you made changes in source code and want to rebuild
    d. `ros2 run `

## Notes

### QoS Profile

        static const rmw_qos_profile_t qos_services = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,  // message queue depth
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};

### Notes

- Default rmw implementation fastdds is used.
- Tested inter-container communication:
    ```bash
    Publisher container
    root@1292b466a8ff:/# ros2 multicast send
    Sending one UDP multicast datagram...

    Subscriber container
    root@4422c32bed4f:/# ros2 multicast receive
    Waiting for UDP multicast datagram...
    Received from 172.19.0.3:59478: 'Hello World!'
    ```

- Default time syncronization between containers (since they use same clock source on device) has satisfactory performance as seen in timestamps:
    ```bash
    temperature_subscriber  | [INFO] [1766352970.924583734] [temperature_subscriber_node]: Received temperature: 23.45  from sensor 'sensor_1' | Moving Avg: 21.68 | Min: 20.00 | Max: 30.00 | Trend: rising
    temperature_publisher   | [INFO] [1766352970.925232009] [temperature_publisher_node]: Publishing: "23.44651357452895", Sensor ID: "sensor_1"
    ```
    Note: the subscriber logs first since the publisher node prints after the message is already published.

## Resources used

- Folder structure: https://github.com/WATonomous/wato_monorepo/blob/main/docs/monorepo.md

# TODO

- Add a third container with a web-based visualization (ROS2 bridge to WebSocket)
- docs