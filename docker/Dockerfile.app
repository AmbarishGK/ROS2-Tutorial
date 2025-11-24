FROM ambarishgk007/ros2-humble-rviz-gazebo:jammy-ml-cpu

USER ros
WORKDIR /home/ros/ROS2-Tutorial

# Copy sources and data
COPY --chown=ros:ros examples/src/ros2_yolo_example  ./examples/src/ros2_yolo_example
COPY --chown=ros:ros examples/data                   ./examples/data

# Clean any stale workspace folders inside the image (just in case)
RUN rm -rf /home/ros/ROS2-Tutorial/examples/build /home/ros/ROS2-Tutorial/examples/install /home/ros/ROS2-Tutorial/examples/log || true

# Verify colcon sees the package and build it
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              cd /home/ros/ROS2-Tutorial/examples && \
              colcon list && \
              colcon build --event-handlers console_direct+"

# Auto-source the overlay
RUN echo 'source /home/ros/ROS2-Tutorial/examples/install/setup.bash' >> /home/ros/.bashrc

# Convenience envs
ENV YOLO_MODEL=/home/ros/ROS2-Tutorial/examples/data/yolov8n.pt \
    YOLO_SAMPLE=/home/ros/ROS2-Tutorial/examples/data/sample.mp4

ENTRYPOINT ["/bin/bash","-lc"]
CMD ["bash"]
