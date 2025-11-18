# 🧭 ROS 2 + Docker Cheatsheet (Humble / Jammy)

A quick reference for running, managing, and testing your ROS 2 environment inside the  
[`ambarishgk007/ros2-humble-rviz-gazebo:jammy`](https://hub.docker.com/r/ambarishgk007/ros2-humble-rviz-gazebo) Docker image.

---

## 🐳 Docker Commands

### 🔹 Pull & Run the Container
```bash
# Pull the prebuilt image (multi-arch: amd64 + arm64)
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy

# Run interactively (Linux example)
xhost +local:
docker run -it --rm --name ros2course   -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### 🔹 Container Management
```bash
docker ps                    # list running containers
docker exec -it ros2course bash   # open a new terminal inside the same container
docker stop ros2course       # stop container manually
docker rm ros2course         # remove stopped container
docker images                # list downloaded images
docker rmi <image_id>        # remove an image
```

### 🔹 Build Your Own (optional)
```bash
docker build -t my-ros2-image:jammy -f docker/Dockerfile .
```

---

## 🧠 Inside the Container

### 🔹 Environment Setup
```bash
source /opt/ros/humble/setup.bash
```

> ✅ Tip: In this image, the ROS 2 environment is already auto-sourced in `.bashrc`.

### 🔹 Quick Sanity Tests
```bash
rviz2               # Launch RViz2 GUI
gazebo              # Launch Gazebo Classic
xeyes               # (tiny X11 test window)
glxinfo -B          # Check OpenGL renderer info
```

---

## 🚀 ROS 2 Core Commands

### 🔹 Nodes, Topics, Services
```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### 🔹 Inspecting Data
```bash
ros2 topic echo /topic_name
ros2 topic info /topic_name
ros2 interface show geometry_msgs/msg/Twist
```

### 🔹 Launching Demos
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
ros2 launch demo_nodes_cpp talk_listen.launch.py
```

---

## 🧩 Building and Packages

### 🔹 Create a Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_pkg
```

### 🔹 Build & Source
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🧾 Parameters, Bags, and TF

### 🔹 Parameters
```bash
ros2 param list
ros2 param get /node_name param_name
ros2 param set /node_name param_name value
```

### 🔹 rosbag2 Recording
```bash
ros2 bag record -a                      # record all topics
ros2 bag record /topic1 /topic2         # record selected topics
ros2 bag info <bag_name>
ros2 bag play <bag_name>
```

### 🔹 TF2 Utilities
```bash
ros2 run tf2_tools view_frames          # generate TF tree PDF
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## 🧰 Useful Shortcuts

| Task | Command |
|------|----------|
| Open new terminal in same container | `docker exec -it ros2course bash` |
| Save container state as image | `docker commit ros2course my-snapshot:latest` |
| Clean all stopped containers | `docker container prune` |
| Clean dangling images | `docker image prune` |

---

## ⚡ Troubleshooting

| Problem | Solution |
|----------|-----------|
| GUI apps not opening (Linux) | Run `xhost +local:` before `docker run` |
| GUI apps not opening (Windows WSL2) | Run container *inside* WSL; check `/mnt/wslg` exists |
| macOS blank window | Ensure XQuartz is running (`xhost + 127.0.0.1`) |
| Permission denied (Docker) | Add user to Docker group → `sudo usermod -aG docker $USER` |

---

**Author:** [Ambarish G.K](https://github.com/ambarishgk)  
**Docker Hub:** [ambarishgk007/ros2-humble-rviz-gazebo](https://hub.docker.com/r/ambarishgk007/ros2-humble-rviz-gazebo)
