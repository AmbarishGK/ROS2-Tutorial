# ROS 2 Connected Course (Humble, Jammy)

A **fully explained, classroom-ready** ROS 2 course designed to run seamlessly on  
**Linux**, **Windows (WSL2 + WSLg)**, and **macOS (XQuartz)** — all using **one Docker image**.

> This repository contains tutorials, example code, and a Docker environment to help you learn and experiment with **ROS 2 Humble Hawksbill** and related tools (RViz2, Gazebo Classic, colcon, rosdep, etc.).

---

## 📘 Course Overview

This course is built to teach ROS 2 step-by-step with hands-on examples.  
It is ideal for students, researchers, and developers who want a ready-to-run simulation and visualization setup.

- **Start with Tutorial 0** → Pull and run the Docker container.
- **Then follow Tutorials 1–13** in order for progressive learning.
- Works on all major platforms via containerization.

---

## 🧩 Repository Structure

```
ROS2-Tutorial/
│
├── docker/                 # Dockerfile to build the ROS 2 environment
│   └── Dockerfile
│
├── docs/                   # MkDocs tutorials (source for website)
│   ├── 0_getting_started.md
│   ├── 1_nodes_topics_services.md
│   ├── 2_parameters_and_launchfiles.md
│   ├── ...
│   ├── 13_multi_robot_and_networking.md
│   └── index.md
│
├── examples/               # (Optional) example scripts or code used in docs
│
├── mkdocs.yml              # MkDocs configuration (for hosted documentation)
├── requirements.txt        # Python dependencies for docs build
├── README.md               # This file
└── LICENSE
```

The documentation is automatically deployed via **GitHub Pages** (see [Online Tutorials](#-online-tutorials)).

---

## 🐳 Docker Image Information

Pre-built multi-architecture image available on Docker Hub:

**Repository:**
```
ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

**Pull command:**
```bash
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

The image automatically detects and runs on:
- `amd64` (Intel / AMD laptops & desktops)
- `arm64` (Apple Silicon, Jetson boards, etc.)

No authentication or keys are needed — the image is public.

---

## ⚙️ Installing Docker

### Linux (Ubuntu / Debian)
```bash
sudo apt update
sudo apt install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu   $(. /etc/os-release && echo "$VERSION_CODENAME") stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
Then add yourself to the Docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Windows 11 / 10
1. Install **Docker Desktop** from:  
   👉 [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Enable **WSL2** and install Ubuntu from the Microsoft Store.
3. Open Ubuntu (WSL2) and follow the Linux quick start below.

### macOS
1. Install **Docker Desktop for Mac**:  
   👉 [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. For GUI apps (RViz, Gazebo), install **XQuartz**:  
   👉 [https://www.xquartz.org/](https://www.xquartz.org/)
3. Restart your Mac after installation.

---

## 🚀 Quick Start Guide

### 🐧 Linux (X11)
```bash
xhost +local:
docker run -it --rm --name ros2course   -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### 🪟 Windows 11 (WSL2 + WSLg)
Run this inside your WSL Ubuntu shell:
```bash
docker run -it --rm --name ros2course   -e DISPLAY=$DISPLAY   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR   -e PULSE_SERVER=$PULSE_SERVER   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /mnt/wslg:/mnt/wslg   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri --device /dev/dxg   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### 🍎 macOS (with XQuartz)
```bash
xhost + 127.0.0.1
docker run -it --rm --name ros2course   -e DISPLAY=host.docker.internal:0   -e QT_X11_NO_MITSHM=1   -v $(pwd):/home/ros/ros2_tutorial   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

---

## 🧠 Using the Container

Open a **new terminal** into the same container:
```bash
docker exec -it ros2course bash
source /opt/ros/humble/setup.bash
```

**Inside the container you can run:**
```bash
rviz2                # Open RViz2 GUI
gazebo               # Open Gazebo Classic
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

---

## 🔍 Verify GUI
```bash
xeyes        # tiny X11 test window
glxinfo -B   # check renderer info (mesa-utils)
```

---

## 📚 Tutorials
All tutorials are in the `docs/` folder, and rendered online at:
👉 [https://ambarishgk007.github.io/ROS2-Tutorial/](https://ambarishgk007.github.io/ROS2-Tutorial/)

| Tutorial | Topic |
|-----------|--------|
| 0 | Getting Started |
| 1 | Nodes, Topics, and Services |
| 2 | Parameters and Launch Files |
| 3 | Create a Package (Pub/Sub) |
| 4 | Services and Actions |
| 5 | TF2 and RViz2 |
| 6 | URDF and Robot Description |
| 7 | Gazebo – Empty World & Spawn |
| 8 | Gazebo Sensors and Plugins |
| 9 | Navigation (Nav2) |
| 10 | Record & Replay (rosbag2) |
| 11 | RQT, RViz, and TF Debugging |
| 12 | Custom Interfaces and Lifecycle |
| 13 | Multi-Robot and Networking |

---

## 🧭 Checking Repository & Image Visibility

### Docker Image
To confirm the image is public:
```bash
docker logout
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy
```
If it pulls successfully, it’s public.  
(Or check Docker Hub → Settings → “Repository Visibility: Public.”)

### GitHub Repository
On GitHub → your repo home → under the name, you’ll see a **Public** or **Private** badge.

If you need to change it:
**Settings → General → Danger Zone → Change repository visibility → Public.**

---

## 🧩 Support & Contribution

- To fix typos or contribute new tutorials, open a **Pull Request**.  
- To rebuild the Docker image locally:
  ```bash
  docker build -t ambarishgk007/ros2-humble-rviz-gazebo:jammy -f docker/Dockerfile .
  ```

---

## 🏁 Summary

✔️ Prebuilt ROS 2 Humble environment with GUI tools  
✔️ Works across Linux, macOS, and Windows  
✔️ Includes all core tutorials 0–13  
✔️ Open-source and free to use for learning

Enjoy your journey through **ROS 2 Connected Course!**

---

**Author:** [Ambarish G.K](https://github.com/ambarishgk007)  
**Docker Hub:** [ambarishgk007/ros2-humble-rviz-gazebo](https://hub.docker.com/r/ambarishgk007/ros2-humble-rviz-gazebo)
