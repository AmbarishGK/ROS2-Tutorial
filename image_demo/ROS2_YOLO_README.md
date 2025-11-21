# ROS 2 YOLO Demo (Humble)

Pre-baked ROS 2 workspace with:
- `ros2demo`: simple pub/sub “Hello” nodes  
- `image_demo`: video publisher, YOLOv8 detector, RViz visualization  
- Docker image that runs the full demo (no extra installs needed)

Tested on Ubuntu 22.04 with Docker and X11.

---

## 📦 Repo layout

```
ROS2-Tutorial/
├── docker/
│   ├── Dockerfile.yolo
│   └── entrypoint_yolo.sh
├── examples/
│   └── data/
│       ├── sample.mp4
│       └── yolov8n.pt
├── image_demo/        # YOLO & image nodes (+ launch & RViz)
│   ├── image_demo/*.py
│   ├── launch/yolo_demo.launch.py
│   ├── rviz/yolo_image.rviz
│   └── resource/image_demo
├── ros2demo/          # hello publisher/subscriber
└── README.md
```

---

## ✅ Prerequisites

- Docker installed
- X11 available (Linux desktop).  
  One-time per session:
  ```bash
  xhost +local:root
  ```

> If you’re on Wayland, you may need to run an Xorg session or adapt the display sharing method.

---

## 🚀 Quickstart: Build the image

From the repo root:

```bash
docker build -f docker/Dockerfile.yolo -t ros2-yolo:humble .
```

This image includes:
- ROS 2 Humble Desktop (rviz2, image transport)
- `cv_bridge`, `vision_msgs`
- Python venv with `opencv-python`, `ultralytics` (+ torch, torchvision)
- Your packages pre-built
- A launch file that starts YOLO + RViz

---

## ▶️ Run the baked demo (no mounts needed)

```bash
xhost +local:root

docker run --rm -it   --env DISPLAY=$DISPLAY   --env QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --name ros2-yolo   ros2-yolo:humble
```

What it does:
- Starts `video_yolo` reading `/ws/src/ROS2-Tutorial/examples/data/sample.mp4`
- Runs YOLOv8n (`yolov8n.pt`)
- Publishes annotated frames to `/image_yolo`
- Opens RViz showing `/image_yolo`

---

## 🧳 Use your own data (mount & override)

Mount your folder and pass launch args:

```bash
docker run --rm -it   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/examples/data:/data   --name ros2-yolo   ros2-yolo:humble   ros2 launch image_demo yolo_demo.launch.py     video_path:=/data/sample.mp4     model_path:=/data/yolov8n.pt     conf_thres:=0.35
```

- `video_path`: any `.mp4` or OpenCV-readable video
- `model_path`: any Ultralytics YOLOv8 `.pt` (e.g., `yolov8n.pt`, `yolov8s.pt`)
- `conf_thres`: detection confidence threshold

---

## ⚡ Use GPU (optional)

If you have NVIDIA drivers + NVIDIA Container Toolkit, you can pass the GPU to the container.  
> Note: the default image includes **CPU** PyTorch. For full CUDA acceleration, you’d build a CUDA-enabled variant (ask if you want a CUDA Dockerfile). Still, `--gpus all` is harmless and enables hardware GL paths.

```bash
docker run --rm -it --gpus all   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --name ros2-yolo   ros2-yolo:humble
```

For better OpenGL (RViz) on Intel/AMD:
```bash
# grant access to DRM devices
docker run --rm -it   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --device /dev/dri:/dev/dri   --name ros2-yolo   ros2-yolo:humble
```

---

## 🧪 Headless / no RViz

If you only want the topics (no GUI):

```bash
docker run --rm -it   ros2-yolo:humble   ros2 run image_demo video_yolo --ros-args     -p video_path:=/ws/src/ROS2-Tutorial/examples/data/sample.mp4     -p model_path:=/ws/src/ROS2-Tutorial/examples/data/yolov8n.pt     -p conf_thres:=0.25
```

Then on the host (with `ros-humble-desktop` installed) you can run your own RViz or echo topics via a ROS bridge/network.

---

## 🔎 Topics

- Annotated frames: `/image_yolo` (`sensor_msgs/Image`)
- Structured detections (if `vision_msgs` available): `/detections_yolo` (`vision_msgs/Detection2DArray`)
- Fallback JSON detections: `/detections_yolo_json` (`std_msgs/String`)

List & inspect:
```bash
ros2 topic list
ros2 topic echo /detections_yolo
```

---

## 📄 License & models

- Code: see `LICENSE`.  
- YOLO model weights (`yolov8n.pt`) are from Ultralytics; check their license. Replace with your own weights if needed.

---

### Build & Run Cheatsheet

```bash
# Build
docker build -f docker/Dockerfile.yolo -t ros2-yolo:humble .

# Run baked demo
xhost +local:root
docker run --rm -it   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   ros2-yolo:humble

# Run with your data
docker run --rm -it   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/examples/data:/data   ros2-yolo:humble   ros2 launch image_demo yolo_demo.launch.py     video_path:=/data/sample.mp4     model_path:=/data/yolov8n.pt     conf_thres:=0.35
```
