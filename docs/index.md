# ROS 2 Connected Course (Humble, Jammy)

This is a **fully explained, classroom‑ready** ROS 2 course that runs on Linux, Windows (WSL2 + WSLg), and macOS (XQuartz) using one Docker image.

- Start with **Tutorial 0** to pull and run the container, and to learn how to open **new terminals** in the same container.
- Then follow Tutorials 1–13 in order.

## Quick start (Linux example)
```bash
xhost +local:
docker run -it --rm --name ros2course   -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   -v $(pwd):/home/ros/ros2_tutorial   --device /dev/dri   ambarishgk007/ros2-humble-rviz-gazebo:jammy
```
New terminal for the same container:
```bash
docker exec -it ros2course bash
source /opt/ros/humble/setup.bash
```
