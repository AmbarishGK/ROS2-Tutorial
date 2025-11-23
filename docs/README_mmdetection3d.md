# MMDetection3D LiDAR Demo (Docker)

This tutorial shows how to run a **PointPillars** LiDAR object-detection demo from **MMDetection3D** using a prebuilt Docker image, with GUI display support on Linux, macOS, and Windows.

> **Docker image**
>
> `docker pull ambarishgk007/mmdetect-cuda118:latest`  
> (CUDA 11.8, Torch 2.1.2, mmcv 2.1.0, mmdet 3.2.0, mmdet3d 1.4.0, NumPy 1.26.4)

---

## 1) Enable GUI (so `--show` opens a window)

You can run headless (skip these steps and omit `--show`) or enable GUI per OS.

### Linux (X11; Ubuntu desktop)
```bash
# allow the container to access your X server
xhost +local:root
```
If you use Wayland, ensure XWayland is installed and consider adding `-e QT_X11_NO_MITSHM=1` in the `docker run` command.

### macOS (Docker Desktop + XQuartz)
1) Install **XQuartz**, open *Preferences → Security* and enable **Allow connections from network clients**.  
2) Start XQuartz, then:
```bash
xhost + 127.0.0.1
```
We will use `DISPLAY=host.docker.internal:0` in the `docker run` command.

### Windows (Docker Desktop + VcXsrv)
1) Install and launch **VcXsrv** (Multi-window or One large window, **Disable access control**).  
2) We will use `DISPLAY=host.docker.internal:0.0` in the `docker run` command.

---

## 2) Run the container

### Linux (X11)
```bash
docker run --name mmdetection3d-1 \
  --gpus all \
  --shm-size=8g \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/workspace \
  -w /workspace \
  -it ambarishgk007/mmdetect-cuda118:latest bash
```
> If you need ROS 2 local discovery across the host network, you may add `--network host` (Linux only).

### macOS
```bash
docker run --name mmdetection3d-1 \
  --gpus all \
  --shm-size=8g \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v "$PWD":/workspace \
  -w /workspace \
  -it ambarishgk007/mmdetect-cuda118:latest bash
```

### Windows (PowerShell)
```powershell
docker run --name mmdetection3d-1 `
  --gpus all `
  --shm-size=8g `
  -e DISPLAY=host.docker.internal:0.0 `
  -e QT_X11_NO_MITSHM=1 `
  -v "%cd%":/workspace `
  -w /workspace `
  -it ambarishgk007/mmdetect-cuda118:latest bash
```

---

## 3) Run the MMDetection3D demo

Inside the container:
```bash
cd /workspace/mmdetection3d

# Show a window (if GUI is configured) and also save outputs/
python demo/pcd_demo.py \
  demo/data/kitti/000008.bin \
  checkpoints/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py \
  checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth \
  --device cuda:0 \
  --pred-score-thr 0.3 \
  --out-dir outputs \
  --show
```

- **Headless?** Remove `--show`. Images/meshes will still be saved in `outputs/`.
- You can try other configs in `configs/` and compatible checkpoints in `checkpoints/`.

---

## 4) Troubleshooting

- **No window appears**
  - Linux: ensure `xhost +local:root` and the `/tmp/.X11-unix` bind mount are present.
  - macOS: confirm XQuartz is running and `DISPLAY=host.docker.internal:0` was set.
  - Windows: ensure VcXsrv is running with *Disable access control*; use `DISPLAY=host.docker.internal:0.0`.

- **NumPy ABI error mentioning “NumPy 2.x”**
  The image pins NumPy to **1.26.4**. If it was upgraded by another package, reset it:
  ```bash
  python -m pip install --no-cache-dir "numpy==1.26.4"
  ```

- **OpenCV GL errors**
  The image ships `opencv-python-headless`. To use full OpenCV GUI instead, install system GL inside the container:
  ```bash
  apt-get update && apt-get install -y libgl1 libglib2.0-0
  ```

- **MMCV CUDA ops error (`mmcv._ext`)**
  The image already contains a matching wheel for cu118. If you change PyTorch or CUDA, reinstall MMCV with the proper prebuilt wheel for your new Torch/CUDA pair.

---

## 5) Clean up / re-enter

```bash
# leave
exit

# re-enter same container
docker start -ai mmdetection3d-1

# delete it when done
docker rm -f mmdetection3d-1
```

---

## MkDocs Navigation Snippet

Add this to your `mkdocs.yml` so the page appears under **Examples → MMDetection3D**:

```yaml
nav:
  - Home: index.md
  - Cheatsheet: CHEATSHEET.md
  - Tutorials:
      - 0. Getting Started: 0_getting_started.md
      # ...
  - Examples:
      - YOLO: yolo_example.md
      - MMDetection3D: examples/mmdetection3d.md
```
