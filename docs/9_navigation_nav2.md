# Tutorial 9: Navigation (Nav2) in Simulation

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**Nav2** moves a robot to a goal using mapping, planning, and control. It consumes sensor topics and TF.

## Launch Nav2 (with sim time)
```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True
```
Send a goal:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Why this matters
Autonomy relies on accurate transforms and sensor data to plan safe paths.

## How you’ll use it
You’ll integrate your URDF + Gazebo sensors with Nav2 for closed-loop navigation.

## Wrap‑up
- You started Nav2 and sent a goal.
- You saw how actions control long-running tasks.
- You integrated sim time with the stack.

**Next:** Tutorial 10 - rosbag2
