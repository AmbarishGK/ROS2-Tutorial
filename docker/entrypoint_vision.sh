#!/usr/bin/env bash
set -e

# Source ROS and (optionally) the built workspace
source /opt/ros/${ROS_DISTRO}/setup.bash || true
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

MODE="${1:-yolo}"

if [[ "${MODE}" == "yolo" ]]; then
  # Run the pre-baked YOLO+RViz launch if present in the workspace
  if command -v ros2 >/dev/null 2>&1; then
    if ros2 pkg list | grep -q "^image_demo$"; then
      exec ros2 launch image_demo yolo_demo.launch.py \
        video_path:=/ws/src/ROS2-Tutorial/examples/data/sample.mp4 \
        model_path:=/ws/src/ROS2-Tutorial/examples/data/yolov8n.pt \
        conf_thres:=0.25
    else
      echo "[entrypoint] image_demo package not found in /ws. Dropping to bash."
      exec bash
    fi
  else
    echo "[entrypoint] ros2 not found. Dropping to bash."
    exec bash
  fi

elif [[ "${MODE}" == "mmdet3d-demo" ]]; then
  # Example mmdetection3d demo (requires sample point cloud path)
  shift || true
  DEMO_PCD="${1:-/workspace/mmdetection3d/tests/data/kitti/000000.bin}"
  CFG="configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py"
  CKPT="/workspace/mmdetection3d/checkpoints/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class_20210831_063303-3d3f69ab.pth"
  if [ -f "/workspace/mmdetection3d/tools/analysis_tools/vis.py" ]; then
    cd /workspace/mmdetection3d
    # Use the simple demo script if available; fallback to pcd_demo.py/vis.py variants by version
    if [ -f "demo/pcd_demo.py" ]; then
      python demo/pcd_demo.py \
        ${CFG} ${CKPT} ${DEMO_PCD} --device cuda:0
    else
      echo "[entrypoint] Please run mmdet3d demo manually (script path differs by version)."
      exec bash
    fi
  else
    echo "[entrypoint] mmdetection3d not found. Dropping to bash."
    exec bash
  fi

else
  # Execute any custom command passed
  exec "$@"
fi
