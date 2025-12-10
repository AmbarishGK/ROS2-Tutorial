# ROS 2 Jetson Vision Suite  
### YOLOv8 (2D) + MMDetection3D (3D LiDAR) + Core ROS 2 Demos  
_All running inside a single GPU-accelerated Jetson container_

This repository contains a ROS 2 Humble workspace that integrates:

| Package | Description |
|--------|-------------|
| **mmdetect_lidar_demo** | Real-time 3D LiDAR detection using MMDetection3D (PointPillars, SECOND, CenterPoint, etc.) |
| **image_demo** | YOLOv8 2D detection pipeline over images/video |
| **ros2demo** | Basic ROS 2 publisher/subscriber examples |

All packages are fully tested on Jetson using the container:

```
ambarishgk007/jetson-mmdetetc3d-pth12.2-torch2.1:latest 
```

Built with:
- JetPack 6 / L4T r36.x  
- CUDA 12.2  
- PyTorch 2.1.0  
- MMCV 2.1.0  
- MMDetection 3.2.0  
- MMDetection3D 1.4.0  
- YOLOv8  
- ROS 2 Humble  

---

# 1. Running the Container

## 1.0 Pull the image

```bash
sudo docker pull ambarishgk007/ambarishgk007/jetson-ros2-mmdetetc3d-yolo-pth12.2-torch2.1:latest
```

## 1.1 Start the container

```bash
xhost +local:root

sudo docker run --name ros2-vision-suite   --runtime nvidia   --gpus all   --network host   --shm-size=8g   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -it ambarishgk007/ambarishgk007/jetson-ros2-mmdetetc3d-yolo-pth12.2-torch2.1:latest  bash
```

Inside container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ROS2-Tutorial
source install/setup.bash
```

---

# 2. ROS 2 Workspace Layout

```
ROS2-Tutorial/
  ├── mmdetect_lidar_demo/
  ├── image_demo/
  ├── ros2demo/
  ├── checkpoints/
  ├── examples/data/
  └── install/, build/, log/
```

---

# 3. LiDAR Detection Demo

Run rosbag:

```bash
ros2 bag play multilidarcalib12
```

Start detector:

```bash
ros2 run mmdetect_lidar_demo mmdetect_lidar_demo
```

RViz setup:

- `/velodyne_points`
- `/bounding_box_edges`
- `/lidar_bounding_boxes`

---

# 4. Switching MMDetection3D Models

Example:

```bash
ros2 run mmdetect_lidar_demo mmdetect_lidar_demo   --ros-args   -p config_file:=/workspace/mmdetection3d/configs/second/...py   -p checkpoint_file:=/workspace/mmdetection3d/checkpoints/...pth
```

---

# 5. YOLOv8 Image Demo

Run:

```bash
ros2 launch image_demo yolo_demo.launch.py
```

View in RViz or rqt:

```bash
rqt_image_view
```

Topic: `/image_yolo`

---

# 6. Basic ROS 2 Demo

Publisher:

```bash
ros2 run ros2demo ros2demo_publisher
```

Subscriber:

```bash
ros2 run ros2demo ros2demo_subscriber
```

---

# 7. Running Full Perception Stack

YOLO (2D):

```bash
ros2 launch image_demo yolo_demo.launch.py
```

LiDAR 3D:

```bash
ros2 run mmdetect_lidar_demo mmdetect_lidar_demo
```

Bag:

```bash
ros2 bag play multilidarcalib12
```

RViz:

```bash
rviz2
```

Add displays:
- `/velodyne_points`
- `/bounding_box_edges`
- `/lidar_bounding_boxes`
- `/image_yolo`

---

<!-- # 8. Summary

- ✔ YOLOv8 real-time image detection  
- ✔ MMDetection3D real-time LiDAR 3D boxes  
- ✔ GPU-accelerated inference  
- ✔ ROS 2 Humble integration  
- ✔ Ready for autonomous robotics development   -->
