# docker/Dockerfile.app
# App image that bakes your examples into the ML-CPU image
FROM ambarishgk007/ros2-humble-rviz-gazebo:jammy-ml-cpu

USER root
# Create workspace
RUN mkdir -p /home/ros/ROS2-Tutorial/examples/src && chown -R ros:ros /home/ros/ROS2-Tutorial

USER ros
WORKDIR /home/ros/ROS2-Tutorial/examples

# Copy only source, launch, params, data (no build artifacts)
# Adjust paths if you rename your package later.
COPY --chown=ros:ros examples/src/ros2_yolo_example ./src/ros2_yolo_example
COPY --chown=ros:ros examples/src/ros2_yolo_example/launch ./src/ros2_yolo_example/launch
COPY --chown=ros:ros examples/src/ros2_yolo_example/params ./src/ros2_yolo_example/params
# Optional data (& model/video) if you want them pre-bundled
WORKDIR /home/ros/ROS2-Tutorial
COPY --chown=ros:ros yolov8n.pt ./yolov8n.pt
COPY --chown=ros:ros sample.mp4 ./sample.mp4
COPY --chown=ros:ros examples/data ./examples/data

# Build the example package
SHELL ["/bin/bash","-lc"]
RUN source /opt/ros/humble/setup.bash && \
    cd /home/ros/ROS2-Tutorial/examples && \
    rm -rf build install log && \
    colcon build --symlink-install && \
    echo 'source /home/ros/ROS2-Tutorial/examples/install/setup.bash' >> /home/ros/.bashrc

# Convenience helper to get into a ROS env quickly
RUN printf '%s\n' \
  '#!/bin/bash' \
  'set -e' \
  'source /opt/ros/humble/setup.bash' \
  'source /home/ros/ROS2-Tutorial/examples/install/setup.bash' \
  'exec "$@"' \
  | sudo tee /usr/local/bin/ros2env >/dev/null && sudo chmod +x /usr/local/bin/ros2env

# Default shell
ENTRYPOINT ["/bin/bash","-lc"]
CMD ["bash"]
