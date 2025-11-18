# Tutorial 6: URDF and Robot Description

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**URDF** (XML) describes your robot’s links and joints. This is the model that RViz/Gazebo and many tools consume.

## Inspect the example
Open:
```
/home/ros/ros2_tutorial/examples/simple_robot.urdf
```

## Visualize in RViz2
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/ros/ros2_tutorial/examples/simple_robot.urdf
```

## Why this matters
A robot description is the single source of truth for geometry and kinematics.

## How you’ll use it
You’ll add sensors/links here and reuse it in simulation and real robots.

## Wrap‑up
- You learned what URDF is and opened a sample model.
- You viewed the model in RViz2.
- You’re ready to take it into a simulator.

**Next:** Tutorial 7 - Gazebo: Empty World & Spawn
