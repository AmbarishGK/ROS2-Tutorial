# Tutorial 10: Record and Replay with rosbag2

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**rosbag2** records topics to disk for debugging, datasets, and reproducibility.

## Record
```bash
ros2 bag record -o my_bag /tf /tf_static /scan /cmd_vel /odom
```

## Replay
```bash
ros2 bag play my_bag
```

## Why this matters
Time‑shifting lets you debug without rerunning the robot.

## How you’ll use it
You’ll capture scenarios and replay them to test your code deterministically.

## Wrap‑up
- You recorded multiple topics to a bag.
- You replayed data to reproduce scenarios.
- You can now iterate faster on algorithms.

**Next:** Tutorial 11 - RQT, RViz, TF Debugging
