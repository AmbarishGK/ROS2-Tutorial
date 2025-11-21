# YOLO Example (ROS 2 + Docker)

This example shows how to **run a pre-baked YOLOv8 demo** inside a Docker image, view annotated video in RViz, and understand the **node structure** so you can tweak it for your projects. No Docker *build* steps here—just **pull and run** the published image.

> Image used: `docker pull ambarishgk007/ros2-yolo:humble`

---

## What you’ll run

- A ROS 2 node **`video_yolo`** that:
  - Reads a video (`.mp4`) with OpenCV.
  - Runs **YOLOv8** object detection (Ultralytics).
  - Publishes **annotated frames** to `/image_yolo` (`sensor_msgs/Image`).
  - Publishes **detections** (class + confidence + bbox) to:
    - `/detections_yolo` (`vision_msgs/Detection2DArray`) if available **or**
    - `/detections_yolo_json` (`std_msgs/String`) as a JSON fallback.
- **RViz** opens with a pre-configured view that subscribes to `/image_yolo`.

---

## Prerequisites

- Linux desktop with **Docker** and X11.
- Allow X11 access (once per session):
  ```bash
  xhost +local:root
  ```

---

## Quickstart (pull & run)

Pull the prebaked image and run the example (no mounts required):

```bash
docker pull ambarishgk007/ros2-yolo:humble

docker run --rm -it   --env DISPLAY=$DISPLAY   --env QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --name ros2-yolo   ambarishgk007/ros2-yolo:humble
```

**What happens**

- Launch file starts: `ros2 launch image_demo yolo_demo.launch.py`
- The node reads `/ws/src/ROS2-Tutorial/examples/data/sample.mp4` *inside the image*.
- Detections are rendered on frames → `/image_yolo`.
- **RViz** opens and displays `/image_yolo` automatically.

> If you see OpenGL warnings (e.g., “failed to load driver: iris”), the container will fall back to software rendering. You can ignore these or add `--device /dev/dri:/dev/dri` to try hardware GL on Intel/AMD.

---

## Use your own data (mount + override)

Mount a folder from your host and pass paths via launch args:

```bash
docker run --rm -it   --env DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/examples/data:/data   --name ros2-yolo   ambarishgk007/ros2-yolo:humble   ros2 launch image_demo yolo_demo.launch.py     video_path:=/data/sample.mp4     model_path:=/data/yolov8n.pt     conf_thres:=0.35
```

- `video_path`: any OpenCV-readable video (.mp4, etc.).  
- `model_path`: YOLOv8 model weights (e.g., `yolov8n.pt`).  
- `conf_thres`: detection confidence threshold (0–1).

---

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/image_yolo` | `sensor_msgs/Image` | Annotated frames with bounding boxes |
| `/detections_yolo` | `vision_msgs/Detection2DArray` | Structured detections (if `vision_msgs` present) |
| `/detections_yolo_json` | `std_msgs/String` | JSON list of `{label, conf, bbox_xyxy}` |

List topics / inspect messages:
```bash
ros2 topic list
ros2 topic echo /detections_yolo
```

---

## Node architecture & parameters

### `video_yolo` node (Python, `rclpy`)

**Responsibilities**  
1. Capture frames with OpenCV (`cv2.VideoCapture`).  
2. Run YOLOv8 inference (Ultralytics `YOLO(model_path)`; `predict(conf=...)`).  
3. Draw boxes + labels on a copy of the frame.  
4. Publish annotated `sensor_msgs/Image` via **cv_bridge**.  
5. Publish detections:
   - **Preferred:** `vision_msgs/Detection2DArray` (class + score + bbox).  
   - **Fallback:** JSON list on `std_msgs/String` if `vision_msgs` is not installed.

**Parameters (declared & read via ROS 2 params)**  
- `video_path` (string): absolute path to the input video.  
- `model_path` (string): path to `.pt` weights.  
- `conf_thres` (float): confidence threshold (default 0.25).

**QoS / Frequency**  
- Publishes at the video’s FPS (falls back to ~30 FPS if metadata is odd).  
- QoS depth = 10 on image & detection pubs (good default for RViz).

> The node tries to **loop** the video when it reaches the end.

---

## RViz configuration

The launch file opens RViz with a minimal config that subscribes to `/image_yolo`.  
If you prefer manual control:
```bash
rviz2
```
Then: **Add → Image → Image Topic = `/image_yolo`**.

---

## How the image is structured (for understanding/tweaks)

> You **don’t** need this to run the example—but it helps when customizing.

- Base: `osrf/ros:humble-desktop` (Ubuntu 22.04 + RViz + ROS 2 tools).  
- System ROS packages: `cv_bridge`, `vision_msgs`, `image_transport`.  
- Python deps live in a **virtualenv** at `/opt/venv` (to avoid apt/pip conflicts):  
  - `opencv-python`, `ultralytics` (with `torch`, `torchvision`).  
  - The image sets:
    ```bash
    PATH=/opt/venv/bin:$PATH
    PYTHONPATH=/opt/venv/lib/python3.10/site-packages:$PYTHONPATH
    ```
    so ament/ROS entry points can import Ultralytics.
- Workspace layout inside the container:
  - `/ws/src/ROS2-Tutorial/` → your repo copy (including `image_demo`, `ros2demo`, `examples/data`).  
  - Built with `colcon build --symlink-install`.
- Entrypoint:
  - Sources `/opt/ros/humble/setup.bash` and `/ws/install/setup.bash`.  
- Default `CMD` (what runs when you `docker run` the image):  
  ```bash
  ros2 launch image_demo yolo_demo.launch.py     video_path:=/ws/src/ROS2-Tutorial/examples/data/sample.mp4     model_path:=/ws/src/ROS2-Tutorial/examples/data/yolov8n.pt     conf_thres:=0.25
  ```

---

## Customizing for your needs

- **Change topics**: edit the publishers in `video_yolo.py` (e.g., publish to `/camera/image` to integrate with other stacks).  
- **Different model**: mount or bake your own `.pt` and pass as `model_path`.  
- **Filter classes**: post-filter detections by `label` or `cls_id` before drawing/publishing.  
- **Publish raw frames too**: add another publisher on `/image_raw` (no annotations).  
- **Switch to live camera**: replace `cv2.VideoCapture(video_path)` with a device index (`0`) or GStreamer pipeline.  
- **Structured detections**: prefer `vision_msgs/Detection2DArray` for downstream consumers like tracking/fusion.  
- **RViz layout**: copy the RViz config (under `image_demo/rviz`) and add more panels/views.

---

## Troubleshooting

- **RViz shows warnings about OpenGL**: Often harmless (software rendering). Try `--device /dev/dri:/dev/dri` if available.  
- **No video output**: check you mounted data and passed correct `video_path`.  
- **Slow on CPU**: try `--gpus all` (if you have CUDA support on the host) or use a smaller model (e.g., `yolov8n.pt`).  
- **Can’t see detections**: increase `conf_thres` downwards (e.g., `0.2`) or check topics with `ros2 topic list`.

---
<!-- 
## Add this page to the docs (MkDocs)

1. Create a new file in your repo: `docs/examples/yolo.md` and paste this content.  
2. Update `mkdocs.yml` to add a dropdown **Examples → YOLO**:

```yaml
nav:
  - Home: index.md
  - Tutorials:
      - Getting Started: docs/0_getting_started.md
      # … your other tutorials
  - Examples:
      - YOLO: docs/examples/yolo.md
```

> Adjust paths if your mkdocs structure differs. This page is meant to live under **Examples → YOLO**.

--- -->

## Next steps

- Add a **tracker** node (e.g., SORT/DeepSORT) subscribing to `/detections_yolo`.  
- Publish **segmentation masks** with a segmentation model.  
- Replace video source with a **ROS camera driver** (e.g., ZED/USB cam) and run detections live.

Happy hacking! 🚀
