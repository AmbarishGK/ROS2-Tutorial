# Vision Suite (YOLOv8 + mmdetection3d) — Docker Usage Guide

This guide shows how to use the **combined CUDA Docker image** that bundles:
- **ROS 2 Humble Desktop** (rviz2, cv_bridge, image_transport, vision_msgs, etc.)
- **Ultralytics YOLOv8** for 2D image/video detection
- **OpenMMLab mmdetection3d** (PointPillars example) for 3D LiDAR detection
- Your **ROS 2 workspace** at `/ws` (can be prebaked or mounted for dev)

---

## TL;DR

```bash
# Build the image (from repo root)
docker build -f docker/Dockerfile.vision -t vision-suite:cu118 .

# Run with GPU + X11 for RViz or image windows
xhost +local:root
docker run --rm -it --gpus all   -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --device /dev/dri:/dev/dri   --name vision-suite vision-suite:cu118
```

Inside the container you can run:
- **YOLOv8 on an image/video** (CLI or Python)
- **mmdetection3d demo** on a KITTI point cloud
- **ROS 2 YOLO node** (`image_demo/video_yolo`) to publish annotated images to RViz

---

## Image Layout (Inside Container)

```
/home/ros/
  ├─ mmdetection3d/                # OpenMMLab 3D detection repo (precloned)
  │   ├─ checkpoints/              # Pre-downloaded PointPillars weights
  │   └─ tests/data/kitti/000000.bin  # Sample point cloud
  ├─ Nvidia-Isaac-ROS/             # (Optional) Isaac ROS repo clone
  └─ outputs/                      # Suggested folder for saving results

/ws/                               # ROS 2 colcon workspace
  ├─ src/ROS2-Tutorial/            # Your repo (prebaked or mounted)
  ├─ build/ install/ log/          # Colcon artifacts after build
  └─ ...
/opt/ros/humble/                   # ROS 2 Humble installation
```

> If `/ws/src/ROS2-Tutorial` is **empty**, bind-mount your repo or switch Dockerfile to prebake it (see below).

---

## Run Modes

### 1) **Prebaked workspace** (one-shot use)
If your Dockerfile contains:
```docker
COPY . /ws/src/ROS2-Tutorial
RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /ws && colcon build --symlink-install"
```
then the image already contains the built ROS packages. Just run the container and launch your nodes.

### 2) **Dev mode** (mount sources and build inside)
Use this for rapid iteration:
```bash
docker run --rm -it --gpus all   -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --device /dev/dri:/dev/dri   -v ~/Desktop/thesi/ROS2-Tutorial:/ws/src/ROS2-Tutorial   --name vision-suite vision-suite:cu118 bash

# inside container
source /opt/ros/humble/setup.bash
cd /ws && colcon build --symlink-install
source /ws/install/setup.bash
```

---

## YOLOv8 — Quick Tests (No ROS)

### Detect on a single image
```bash
cd ~
wget -O sample.jpg https://ultralytics.com/images/bus.jpg
yolo detect predict model=yolov8n.pt source=sample.jpg show=True save=True
# results saved in runs/detect/predict/
```

### Detect on a video file
```bash
wget -O sample.mp4 https://samplelib.com/lib/preview/mp4/sample-5s.mp4
yolo detect predict model=yolov8n.pt source=sample.mp4 show=True save=True
```

### Minimal Python snippet
```bash
python3 - <<'PY'
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
res = model("sample.jpg", show=True)
print(res[0].boxes)
PY
```

> If a window fails to open, make sure you ran the container with `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri:/dev/dri`.

---

## mmdetection3d — PointPillars Demo

```bash
# save results in a writable directory
mkdir -p ~/outputs

cd ~/mmdetection3d
python3 demo/pcd_demo.py   configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py   checkpoints/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class_20210831_063303-3d3f69ab.pth   tests/data/kitti/000000.bin   --device cuda:0   --out-dir ~/outputs
```

If you see a permission error writing under the repo, use `~/outputs` or run once:
```bash
sudo chown -R ros:ros ~/mmdetection3d
```

---

## ROS 2 YOLO Node + RViz

`image_demo/video_yolo.py` reads a video/camera, runs YOLO, and publishes:
- `/image_yolo_raw` (original frames)
- `/image_yolo_annotated` (frames with boxes)

```bash
# Build (skip if prebaked)
source /opt/ros/humble/setup.bash
cd /ws && colcon build --symlink-install
source /ws/install/setup.bash

# Get a sample video + weights
cd ~
wget -O sample.mp4 https://samplelib.com/lib/preview/mp4/sample-5s.mp4
wget -O yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt

# Run the node
ros2 run image_demo video_yolo --ros-args   -p video_path:=/home/ros/sample.mp4   -p model_path:=/home/ros/yolov8n.pt   -p conf_thres:=0.25
```

In another terminal:
```bash
rviz2
```
Add **Image** display -> set topic to `/image_yolo_annotated`.

> If `video_yolo` isn’t found: ensure `setup.py` includes  
> `video_yolo = image_demo.video_yolo:main`, then rebuild + `source /ws/install/setup.bash`.

---

## Typical Issues & Fixes

**No GUI / OpenGL errors in RViz**  
Run with: `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri:/dev/dri` and `xhost +local:root` on the host.

**Package found but “No executable found”**  
Add the console script in `image_demo/setup.py`:
```python
entry_points={'console_scripts': [
  'video_yolo = image_demo.video_yolo:main',
]}
```
Rebuild + `source /ws/install/setup.bash`.

**Permission denied writing outputs**  
Use a user-owned directory, e.g. `~/outputs`, or `sudo chown -R ros:ros <path>`.

**ultralytics / torch not found**  
These are pre-installed; if you changed Python envs, verify with
```bash
python3 -c "import ultralytics, torch; print('ok')"
```

---

## Dockerfile Tweaks (Prebake vs Mount)

**Prebake your ROS workspace** (no mounts at runtime):
```docker
COPY . /ws/src/ROS2-Tutorial
RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /ws && colcon build --symlink-install && chown -R ${USERNAME}:${USERNAME} /ws"
```

**Dev mode** (recommended while iterating):
- Mount your repo with `-v ~/Desktop/thesi/ROS2-Tutorial:/ws/src/ROS2-Tutorial`
- Rebuild with `colcon build` inside the container

---

## MkDocs Navigation

Add this page under **Examples** in `mkdocs.yml`:

```yaml
nav:
  - Examples:
      - Vision Suite (YOLO + mmdet3d): examples/vision_suite_guide.md
```

---

## Summary

- Use `vision-suite:cu118` for a single container that runs **YOLOv8**, **mmdetection3d**, and **ROS 2**.
- Choose **prebaked** (stable) or **dev** (flexible) workflow.
- Use RViz to visualize `/image_yolo_annotated` and `mmdetection3d` to test 3D inference.
- Save outputs to `~/outputs` to avoid permission issues.
