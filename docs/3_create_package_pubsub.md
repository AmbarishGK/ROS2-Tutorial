# Tutorial 3: Create a Package and Pub/Sub Nodes (Python)

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
Packages group code, configs, and launch files. The **publisher–subscriber** pattern is the core of ROS 2 messaging.

## Create and build
```bash
mkdir -p ~/ros2_ws/src
cp -r /home/ros/ros2_tutorial/examples/my_first_pkg ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run them
Terminal A:
```bash
ros2 run my_first_pkg publisher_member_function
```
Terminal B:
```bash
ros2 run my_first_pkg subscriber_member_function
```

### Code walkthrough
- `create_publisher(String, 'chatter', 10)`: topic type, name, queue.
- `create_timer(0.5, ...)`: publish every 0.5s.
- `create_subscription(..., 'chatter', ...)`: receives messages and logs them.

## Why this matters
Pub/sub lets you connect many producers and consumers without tight coupling.

## How you’ll use it
Most of your stack (sensors, planners, controllers) communicates with pub/sub.

## Wrap‑up
- You built a package and ran custom nodes.
- You understood basic rclpy APIs.
- You saw messages flow over a topic.

**Next:** Tutorial 4 - Services and Actions
