# Tutorial 13: Multi‑Robot and Networking

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
Multi‑robot work needs unique namespaces, discovery settings, and sometimes separate domains.

## Namespaces
```bash
ros2 run gazebo_ros spawn_entity.py -file /home/ros/ros2_tutorial/examples/simple_robot.urdf -entity robot1 --ros-args -r __ns:=/robot1
ros2 run gazebo_ros spawn_entity.py -file /home/ros/ros2_tutorial/examples/simple_robot.urdf -entity robot2 --ros-args -r __ns:=/robot2
```

## Domains
```bash
export ROS_DOMAIN_ID=7
```

## Test
```bash
ros2 topic echo /robot1/scan
ros2 topic echo /robot2/scan
```

## Why this matters
Avoid topic collisions and control who talks to whom across networks.

## How you’ll use it
Use namespaces and domains to isolate or connect robots in labs and competitions.

## Wrap‑up
- You ran multiple robots with namespaces.
- You controlled discovery with domains.
- You validated cross‑robot topics.

**Next:** You’ve completed the course!
