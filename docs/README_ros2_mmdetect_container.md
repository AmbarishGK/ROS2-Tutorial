# ROS 2 + MMDetection3D LiDAR Detection Container

This container integrates **ROS 2 Humble** with **MMDetection3D** (PointPillars) for real-time 3D object detection from LiDAR data.  
It allows you to subscribe to `/velodyne_points`, perform inference on GPU, and visualize 3D bounding boxes in RViz 2.

---

## 🧱 Base Image

`ambarishgk007/ros2-mmdetect-cuda118:latest`

**Stack inside**
| Component | Version |
|------------|----------|
| Ubuntu | 22.04 LTS |
| CUDA | 11.8 (devel image with `nvcc`) |
| PyTorch | 2.1.2 + cu118 |
| MMCV | 2.1.0 |
| MMDetection | 3.2.0 |
| MMDetection3D | 1.4.0 |
| NumPy | 1.26.4 |
| ROS 2 | Humble Hawksbill (desktop + common perception packages) |

---

## 🚀 Run the Container

### Linux (X11 GUI)
```bash
xhost +local:root
docker run --name ros2-mmdetection3d \
  --gpus all \
  --network host \
  --shm-size=8g \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/workspace \
  -w /workspace \
  -it ambarishgk007/ros2-mmdetect-cuda118:latest bash
```

### macOS (XQuartz) or Windows (VcXsrv)
Use:
```bash
-e DISPLAY=host.docker.internal:0   # macOS
# or
-e DISPLAY=host.docker.internal:0.0 # Windows
```
and ensure your X server is running.

---

## 🧠 Inside the Container

1. **ROS 2 and GPU are ready**  
   ```bash
   source /opt/ros/humble/setup.bash
   nvidia-smi
   ros2 --version
   ```

2. **Model location**
   ```
   /workspace/mmdetection3d/checkpoints/
   ├── pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
   └── hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth
   ```

3. **Node script**  
   `/workspace/Nvidia-Isaac-ROS/Examples/realtimemmdetectboundingbox.py`

---

## ▶️ Run the Node

### 1. Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```

### 2. Start LiDAR publisher
Either connect a Velodyne driver or replay a bag:
```bash
ros2 bag play your_lidar_rosbag
```

### 3. Run detection node
```bash
python3 /workspace/Nvidia-Isaac-ROS/Examples/realtimemmdetectboundingbox.py
```
You’ll see logs:
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
**Add displays**
| Type | Topic | Description |
|------|--------|-------------|
| Marker | `/bounding_box_edges` | Green edges for each 3D box |
| MarkerArray | `/lidar_bounding_boxes` | Semi-transparent green cubes |

Set **Fixed Frame** to your LiDAR’s frame (often `velodyne`).

---

## 🧩 How It Works
1. Subscribes to `/velodyne_points` (`sensor_msgs/PointCloud2`).
2. Converts the point cloud to NumPy `(N, 4)` [x, y, z, intensity].
3. Runs MMDetection3D inference (PointPillars model).
4. Publishes:
   - **`/bounding_box_edges`** → `visualization_msgs/Marker`
   - **`/lidar_bounding_boxes`** → `visualization_msgs/MarkerArray`
5. RViz renders 3D bounding boxes live.

---

## 🧰 Troubleshooting

| Symptom | Fix |
|-----------|-----|
| `FileNotFoundError: pointpillars_...py` | Use absolute paths to config & checkpoint files as shown above. |
| `No boxes detected` | Try a denser cloud or KITTI-format scan. |
| `ModuleNotFoundError: tf_transformations` | Remove that import or `pip install tf-transformations`. |
| No boxes in RViz | Check Fixed Frame matches marker `frame_id`. |
| NumPy ABI error | Re-install `numpy==1.26.4`. |

---

## 🧾 Summary
**Container:** `ambarishgk007/ros2-mmdetect-cuda118:latest`  
**Purpose:** Real-time LiDAR object detection in ROS 2 Humble using MMDetection3D (PointPillars).  
**Outputs:** 3D boxes as RViz markers.  
**Topic input:** `/velodyne_points` (`sensor_msgs/PointCloud2`)  
**Topics output:** `/bounding_box_edges`, `/lidar_bounding_boxes`
