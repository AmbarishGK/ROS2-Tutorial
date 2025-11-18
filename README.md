# 🐋 ROS 2 Humble + RViz2 + Gazebo (Ubuntu 22.04 Jammy)

A cross-platform Docker image with **ROS 2 Humble**, **RViz2**, and **Gazebo Classic** preinstalled.  
Runs seamlessly on **Linux**, **Windows (WSL2/WSLg)**, and **macOS (XQuartz)** — with GUI support for simulation and visualization.

---

## 📦 Features

- 🧠 **ROS 2 Humble Desktop-Full** (`/opt/ros/humble`)
- 🪞 **RViz2**, **Gazebo Classic**, and `gazebo_ros_pkgs`
- ⚙️ **colcon**, **rosdep**, and common build tools
- 🐍 Python 3 + pip + venv
- 🧰 Git, curl, wget, vim, nano, sudo
- 👨‍💻 Non-root `ros` user with passwordless sudo
- 🪄 Auto-sources ROS in every shell
- 🧩 Multi-arch: works on **amd64** (Intel/AMD) and **arm64** (Apple Silicon + Jetson)

---

## 🚀 Quick Start

### 1️⃣ Pull the image
```bash
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### 2️⃣ Run it with GUI support

#### 🐧 Linux (X11)
```bash
xhost +local:

docker run -it --rm   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v $HOME/.Xauthority:/home/ros/.Xauthority:ro   --device /dev/dri   --name ros2sim   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

#### 🪟 Windows 11 (WSL2 + WSLg)
Run from **inside WSL** (e.g. Ubuntu on WSL2):
```bash
docker run -it --rm   -e DISPLAY=$DISPLAY   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR   -e PULSE_SERVER=$PULSE_SERVER   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /mnt/wslg:/mnt/wslg   --device /dev/dri   --device /dev/dxg   --name ros2win   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```
> WSLg provides the windowing layer, so RViz and Gazebo open directly on your Windows desktop.

#### 🍎 macOS (Intel / Apple Silicon + XQuartz)
1. Install **XQuartz**, open Preferences → *Security* → enable **“Allow connections from network clients.”**  
2. Start XQuartz, then in Terminal:
   ```bash
   xhost + 127.0.0.1
   ```
3. Run the container:
   ```bash
   docker run -it --rm      -e DISPLAY=host.docker.internal:0      -e QT_X11_NO_MITSHM=1      --name ros2mac      ambarishgk007/ros2-humble-rviz-gazebo:jammy
   ```
> macOS runs the GUI over XQuartz (TCP X11); performance may be slower since hardware OpenGL isn’t exposed.

---

## 🧩 Inside the Container

You’ll be logged in as the `ros` user.

```bash
# Source environment (already in .bashrc)
source /opt/ros/humble/setup.bash

# Launch tools
rviz2
gazebo
ros2 run demo_nodes_cpp talker
```

### Verify OpenGL + X11
```bash
xeyes        # small X11 test
glxinfo -B   # shows renderer info
```

---

## ⚙️ Advanced Usage

### Persistent workspace
```bash
docker run -it --rm   -v ~/ros2_ws:/home/ros/ros2_ws   -e DISPLAY=$DISPLAY   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### GPU acceleration (Linux NVIDIA)
```bash
docker run -it --rm   --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### USB / sensors (e.g. LIDAR)
```bash
docker run -it --rm   --device=/dev/ttyUSB0   --privileged   -e DISPLAY=$DISPLAY   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

---

## 🧱 Build From Source

If you’d like to build the image yourself:
```bash
git clone https://github.com/ambarishgk007/ros2-humble-rviz-gazebo.git
cd ros2-humble-rviz-gazebo
docker build -t ambarishgk007/ros2-humble-rviz-gazebo:jammy .
```

To build multi-arch (Intel + ARM + Jetson):
```bash
docker buildx build   --platform linux/amd64,linux/arm64   -t ambarishgk007/ros2-humble-rviz-gazebo:jammy   --push .
```

---

## 🧾 License

This image is based on Ubuntu 22.04 (Jammy) and ROS 2 Humble packages  
distributed under the Apache 2.0 and BSD licenses respectively.  
See the [ROS 2 license summary](https://www.ros.org/reps/rep-0149.html) for details.

---

## 💬 Support / Contact

Maintainer: **[Ambarish G K](https://hub.docker.com/u/ambarishgk007)**  
Issues and feature requests welcome via GitHub or Docker Hub comments.

---

**Enjoy fast, reproducible ROS 2 development anywhere!** 🚀
