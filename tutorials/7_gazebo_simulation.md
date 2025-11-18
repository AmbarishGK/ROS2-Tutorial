# Tutorial 7: Simulation with Gazebo

Learn how to simulate your robot in Gazebo.

## 7.1 Install Gazebo
Gazebo Classic is installed with your Docker image.
Run:
```bash
gazebo

```

## 7.2 Spawn a Robot
```bash
ros2 launch gazebo_ros empty_world.launch.py
ros2 run gazebo_ros spawn_entity.py -file simple_robot.urdf -entity my_robot

```

## 7.3 Exercise
✅ Add a wheel joint and observe its rotation in simulation.
