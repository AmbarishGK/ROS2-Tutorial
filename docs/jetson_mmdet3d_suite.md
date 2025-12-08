# Jetson MMDetection3D Suite  
### Torch 2.1 + CUDA 12.2 + MMDetection3D 1.4.0 (PointPillars demo)  
_All inside a single GPU accelerated container for Jetson Orin (JetPack 6 / L4T r36.x)_

This document explains how to use the Jetson MMDetection3D container:

> `ambarishgk007/jetson-mmdetetc3d-pth12.2-torch2.1:latest`

It provides a ready to use environment for 3D LiDAR detection using OpenMMLab:

| Component      | Version / Info                      |
|----------------|-------------------------------------|
| Base image     | `dustynv/l4t-ml:r36.2.0`           |
| CUDA           | 12.2                                |
| PyTorch        | 2.1.0 (Jetson aarch64 build)        |
| NumPy          | 1.26.4 (kept < 2 for ABI safety)    |
| OpenCV         | 4.8.1.78 (Python)                  |
| MMCV           | 2.1.0 (built from source with CUDA) |
| MMEngine       | >= 0.10.4                           |
| MMDetection    | 3.2.0                               |
| MMDetection3D  | 1.4.0 (dev 1.x branch)              |
| Demo           | PointPillars on KITTI pc bin file   |

---

# 1. Pull and Run the Container

## 1.1 Pull

```bash
docker pull ambarishgk007/jetson-mmdetetc3d-pth12.2-torch2.1:latest
```

## 1.2 Run

```bash
docker run --name jetson-mmdet3d   --runtime nvidia   --network host   --shm-size=8g   -it ambarishgk007/jetson-mmdetetc3d-pth12.2-torch2.1:latest   bash
```

---

# 2. Verify the Environment

```bash
python3 - << 'EOF'
import importlib

def safe_import(name):
    try:
        m = importlib.import_module(name)
        print(f"{name}: OK")
        return m
    except Exception as e:
        print(f"{name}: FAIL -> {e}")
        return None

mods = {}
for name in ["numpy", "torch", "cv2", "mmengine", "mmcv", "mmdet", "mmdet3d"]:
    m = safe_import(name)
    if m is not None:
        mods[name] = m

print("\n=== VERSIONS ===")
if "numpy" in mods:
    print("numpy:", mods["numpy"].__version__)
if "torch" in mods:
    print("torch:", mods["torch"].__version__)
    print("cuda version:", mods["torch"].version.cuda)
    print("cuda available:", mods["torch"].cuda.is_available())
if "cv2" in mods:
    print("cv2:", mods["cv2"].__version__)
if "mmengine" in mods:
    print("mmengine:", mods["mmengine"].__version__)
if "mmcv" in mods:
    print("mmcv:", mods["mmcv"].__version__)
if "mmdet" in mods:
    print("mmdet:", mods["mmdet"].__version__)
if "mmdet3d" in mods:
    print("mmdet3d:", mods["mmdet3d"].__version__)

# mmcv CUDA ops test
if "mmcv" in mods:
    from mmcv.ops import nms
    print("\n[OK] mmcv.ops.nms imported")

# simple torch CUDA test
if "torch" in mods:
    import torch
    x = torch.randn(1, 3, 224, 224, device="cuda")
    y = torch.nn.functional.relu(x)
    print("[OK] torch CUDA tensor op:", y.shape)
EOF
```

---

# 3. Run MMDetection3D PointPillars Demo

```bash
cd /root/mmdetection3d

python3 demo/pcd_demo.py   demo/data/kitti/000008.bin   configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py   checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth   --out-dir outputs/pred   --print-result
```

Output JSON will appear in `outputs/pred/`.

---

# 4. Switching Models

Use any config + checkpoint:

```bash
python3 demo/pcd_demo.py <pcd.bin> <config.py> <model.pth> --out-dir outputs/custom --print-result
```

---

# 5. Notes

- Do **not** upgrade torch/numpy/mmcv inside this container.
- This image is meant as a baseline for 3D LiDAR perception on Jetson.
- You may layer ROS 2, Isaac ROS, or custom pipelines on top.

