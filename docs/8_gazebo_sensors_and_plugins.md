# Tutorial 8: Gazebo - Sensors and Plugins

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
Gazebo **plugins** attach simulated sensors/actuators to links. They produce ROS topics just like the real hardware drivers.

## Add a LiDAR
See:
```
/home/ros/ros2_tutorial/examples/simple_robot_with_sensors.urdf
```
Spawn:
```bash
ros2 run gazebo_ros spawn_entity.py   -file /home/ros/ros2_tutorial/examples/simple_robot_with_sensors.urdf   -entity sensor_bot
```

### Watch the data
```bash
ros2 topic list | grep scan
ros2 topic echo /scan
```

## Why this matters
Sensors are the inputs to autonomy; you’ll test algorithms before touching real hardware.

## How you’ll use it
You’ll publish scans/images/IMU data from Gazebo and consume them in your nodes.

## Wrap‑up
- You added a LiDAR plugin and viewed `/scan`.
- You understand plugin-produced topics.
- You can extend the URDF with more sensors.

**Next:** Tutorial 9 - Navigation (Nav2)
