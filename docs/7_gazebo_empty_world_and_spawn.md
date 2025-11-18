# Tutorial 7: Gazebo - Empty World & Spawn

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**Gazebo (classic)** simulates physics and sensors so you can test without hardware.

## Start an empty world
```bash
ros2 launch /home/ros/ros2_tutorial/examples/empty_world.launch.py
```

## Spawn your URDF
```bash
ros2 run gazebo_ros spawn_entity.py   -file /home/ros/ros2_tutorial/examples/simple_robot.urdf   -entity simple_bot
```

### Verify topics
```bash
ros2 topic list
```

## Why this matters
Simulation lets you iterate quickly and safely.

## How you’ll use it
You’ll spawn and respawn robots while developing perception and control.

## Wrap‑up
- You ran Gazebo and spawned your robot.
- You saw simulator topics appear in ROS 2.
- You can change spawn poses and respawn.

**Next:** Tutorial 8 - Gazebo: Sensors and Plugins
