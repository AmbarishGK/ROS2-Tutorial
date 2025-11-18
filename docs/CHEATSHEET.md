# Cheatsheet - Docker & ROS 2

## Docker
```bash
docker pull ambarishgk007/ros2-humble-rviz-gazebo:jammy
docker run -it --rm --name ros2course IMAGE
docker exec -it ros2course bash      # new terminal
docker ps
docker stop ros2course
```

## ROS 2 basics
```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
ros2 service list
rviz2
gazebo
```
