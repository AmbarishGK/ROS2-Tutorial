# ROS 2 + MMDetection3D LiDAR Detection Container

This container bundles **ROS 2 Humble** and **MMDetection3D** (PointPillars) for real‑time 3D object detection from LiDAR data.  
It subscribes to `/velodyne_points`, runs inference on the GPU, and publishes 3D boxes for RViz 2.

---

## 🧱 Image

**Docker Hub:** `ambarishgk007/ros2-mmdetect-cuda118:latest`

**Stack**
| Component       | Version |
|-----------------|---------|
| Ubuntu          | 22.04 LTS |
| CUDA            | 11.8 (devel, with `nvcc`) |
| PyTorch         | 2.1.2 + cu118 |
| MMCV            | 2.1.0 |
| MMDetection     | 3.2.0 |
| MMDetection3D   | 1.4.0 |
| NumPy           | 1.26.4 |
| ROS 2           | Humble (desktop + perception) |

> The image **clones your repo** during build to: `/workspace/Nvidia-Isaac-ROS`  
> and includes **MMDetection3D checkpoints** at: `/workspace/mmdetection3d/checkpoints`.

---

## ⬇️ Pull

```bash
docker pull ambarishgk007/ros2-mmdetect-cuda118:latest
```

---

## 🚀 Run the Container

### Linux (X11 GUI)
```bash
xhost +local:root
docker run --name ros2-mmdetection3d   --gpus all   --network host   --shm-size=8g   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v "$PWD":/workspace   -w /workspace   -it ambarishgk007/ros2-mmdetect-cuda118:latest bash
```

### macOS (XQuartz) / Windows (VcXsrv)
Run an X server, then set DISPLAY:
```bash
# macOS
-e DISPLAY=host.docker.internal:0
# Windows
-e DISPLAY=host.docker.internal:0.0
```

---

## 🧠 What’s Inside

1. **ROS 2 & GPU ready**
   ```bash
   source /opt/ros/humble/setup.bash
   nvidia-smi
   ros2 --version
   ```

2. **Repo & scripts**
   ```
   /workspace/Nvidia-Isaac-ROS/Examples/
     ├── real.py                              # ROS 2 node (recommended)
     └── realtimemmdetectboundingbox.py       # legacy node variant
   ```

3. **Models**
   ```
   /workspace/mmdetection3d/checkpoints/
     ├── pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
     └── hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth
   ```

---

## ▶️ Run the Detection Node

1) **Source ROS 2**
```bash
source /opt/ros/humble/setup.bash
```

2) **Start a LiDAR source**
- Real sensor driver publishing `/velodyne_points`, **or**
- Replay a rosbag:
```bash
ros2 bag play your_lidar_rosbag
```

3) **Run the node**
```bash
python3 /workspace/Nvidia-Isaac-ROS/Examples/real.py
# or
python3 /workspace/Nvidia-Isaac-ROS/Examples/realtimemmdetectboundingbox.py
```
Expected logs:
```
[INFO] [lidar_detection_node]: Loading model on cuda:0…
[INFO] [lidar_detection_node]: Model ready.
[INFO] [lidar_detection_node]: Running inference…
[INFO] [lidar_detection_node]: Inference done.
```

---

## 🖥️ Visualize in RViz 2

```bash
rviz2
```
Add displays:
| Type        | Topic                      | Description                       |
|-------------|----------------------------|-----------------------------------|
| Marker      | `/bounding_box_edges`      | Green line edges of each 3D box   |
| MarkerArray | `/lidar_bounding_boxes`    | Semi‑transparent green cubes      |

Set **Fixed Frame** to your LiDAR frame (e.g., `velodyne`).

---

## 🧩 How It Works

1. Subscribe to `/velodyne_points` (`sensor_msgs/PointCloud2`).  
2. Convert to NumPy `(N,4)` → `[x, y, z, intensity]` (intensity can be zero).  
3. Run **MMDetection3D** PointPillars inference.  
4. Publish detections as:
   - `visualization_msgs/Marker` on `/bounding_box_edges`  
   - `visualization_msgs/MarkerArray` on `/lidar_bounding_boxes`  
5. RViz renders the 3D boxes in real time.

---

## 🧰 Troubleshooting

| Symptom | Fix |
|--------|-----|
| `FileNotFoundError: pointpillars_...py` | Use the absolute config/checkpoint paths shown above. |
| `No boxes detected` | Verify cloud density and coordinate frame; test with KITTI-like scans. |
| `ModuleNotFoundError: tf_transformations` | Remove that import or `pip install tf-transformations`. |
| No markers in RViz | Ensure Fixed Frame equals the LiDAR frame; add Marker/MarkerArray displays. |
| NumPy ABI error | `pip install 'numpy==1.26.4' --force-reinstall` inside the container. |

---

## 🧾 Summary

- **Image:** `ambarishgk007/ros2-mmdetect-cuda118:latest`  
- **Input topic:** `/velodyne_points` (`sensor_msgs/PointCloud2`)  
- **Output topics:** `/bounding_box_edges` (Marker), `/lidar_bounding_boxes` (MarkerArray)  
- **Goal:** Real‑time LiDAR 3D detection (PointPillars) in ROS 2 Humble.
