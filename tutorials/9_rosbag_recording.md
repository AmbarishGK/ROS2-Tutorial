# Tutorial 9: Recording and Replaying Data (rosbag2)

ROS 2 Bag allows you to record and replay topics.

## 9.1 Record Topics
```bash
ros2 bag record -a -o my_bag

```

## 9.2 Replay Data
```bash
ros2 bag play my_bag

```

## 9.3 Exercise
✅ Record only `/turtle1/pose` and replay to visualize motion.
