# Tutorial 5: Coordinate Frames (TF2) and RViz2 Visualization

Robots live in 3D space ; so understanding **TF2** (transform frames) is essential.

---

## 5.1 What is TF2?

TF2 keeps track of where every part of your robot is in space.
Each coordinate frame (like `base_link`, `camera`, or `lidar`) has its position relative to others.

---

## 5.2 Visualizing TF in RViz2

Run `turtlesim` again:
```bash
ros2 run turtlesim turtlesim_node

```

In another terminal:
```bash
rviz2

```

Then:
1. Add → `TF`
2. Add → `Pose` → Topic: `/turtle1/pose`

You’ll see the turtle’s pose updating in RViz.

---

## 5.3 Broadcasting a Static Transform

Example: imagine a camera mounted 10 cm above the robot base.

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link camera_link

```

Now TF knows there’s a `camera_link` frame 10 cm above `base_link`.

---

## 5.4 Inspect Transforms

```bash
ros2 run tf2_ros tf2_echo base_link camera_link

```

Output:
```bash
At time 0.0
- Translation: [0.0, 0.0, 0.1]
- Rotation: (0.0, 0.0, 0.0, 1.0)

```

---

## 5.5 Launching TF Setup Automatically

Use a launch file (`examples/tf_demo.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']
        ),
    ])

```

Run:
```bash
ros2 launch examples/tf_demo.launch.py

```

---

## 5.6 Exercise

✅ Add a `lidar_link` frame offset 20 cm forward.
✅ Visualize both transforms in RViz2.

---

Now you can **visualize and reason about space** ; a key skill for robot navigation.
