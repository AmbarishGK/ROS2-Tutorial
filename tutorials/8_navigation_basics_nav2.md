# Tutorial 8: Navigation Basics (Nav2)

Nav2 allows a robot to move autonomously.

## 8.1 Launch Nav2
```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True

```

## 8.2 Send Navigation Goal
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}"

```

## 8.3 Exercise
✅ Adjust the goal position and observe the path planning.
