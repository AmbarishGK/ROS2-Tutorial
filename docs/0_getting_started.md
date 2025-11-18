# Tutorial 0: Getting Started - Pull, Run, and Use the Course Container

This course uses a prebuilt Docker image so everyone has the **same working environment** on **Linux**, **Windows (WSL2 + WSLg)**, and **macOS (XQuartz)**.

- **Image:** `ambarishgk007/ros2-humble-rviz-gazebo:jammy` (multi‑arch: amd64 + arm64)
- **Container name (we’ll reuse it):** `ros2course`


> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Pull the image (all OS)
```bash
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

## Run the container (choose your OS)

### Linux (X11)
```bash
xhost +local:
docker run -it --rm   --name ros2course   -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### Windows 11 (WSL2 + WSLg) - run inside your WSL Ubuntu
```bash
docker run -it --rm   --name ros2course   -e DISPLAY=$DISPLAY   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR   -e PULSE_SERVER=$PULSE_SERVER   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /mnt/wslg:/mnt/wslg   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri --device /dev/dxg   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

### macOS (with XQuartz)
1) Install + start **XQuartz**, enable **Allow connections from network clients**.  
2) In Terminal:
```bash
xhost + 127.0.0.1
docker run -it --rm   --name ros2course   -e DISPLAY=host.docker.internal:0   -e QT_X11_NO_MITSHM=1   -v $(pwd):/home/ros/ros2_tutorial   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```

## New terminal for the same container
On the host:
```bash
docker exec -it ros2course bash
```

## Sanity checks inside the container
```bash
source /opt/ros/humble/setup.bash
rviz2                # GUI should open
ros2 --help
```


## Why this matters
Containers eliminate 'works on my machine' issues and give you identical ROS 2 + GUI tools anywhere.

## How you’ll use it
You’ll do all tutorials inside this container. Use `docker exec` whenever you need another terminal.

## Wrap‑up
- You can pull and run the course image on any OS.
- You know how to attach more terminals to the same container.
- You verified GUI support (RViz).

**Next:** Go to Tutorial 1 - Nodes, Topics, and Services
