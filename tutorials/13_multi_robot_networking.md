# Tutorial 13: Multi-Robot Networking

Learn how to connect multiple ROS 2 systems.

## 13.1 Check Network
Each robot must be on the same LAN and visible via ping.

## 13.2 Set Environment Variables
```bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=

```

## 13.3 Test Communication
```bash
ros2 topic pub /hello std_msgs/msg/String "data: 'Hi from robot 1'"
ros2 topic echo /hello

```

## 13.4 Exercise
✅ Connect two containers and make one robot respond to the other.
