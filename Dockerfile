# Ubuntu 22.04 (Jammy) + ROS 2 Humble + RViz2 + Gazebo (Classic) + Dev tools
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

# 0) Base OS refresh
RUN apt-get update && \
    apt-get -y upgrade && \
    rm -rf /var/lib/apt/lists/*

# 1) Essentials, GUI/X11 + OpenGL, dev tools, Python toolchain
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata ca-certificates gnupg2 lsb-release software-properties-common \
    build-essential git curl wget vim nano \
    python3 python3-pip python3-venv \
    # X11 + OpenGL for GUI apps (RViz2/Gazebo)
    x11-apps mesa-utils libgl1 libgl1-mesa-dri \
    # Networking convenience
    iputils-ping net-tools \
    # Useful CLI extras
    sudo less \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# 2) ROS 2 APT repo & key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# 3) Install ROS 2 Humble Desktop + Gazebo Classic + build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# 4) Initialize rosdep
RUN set -e; \
    if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi; \
    rosdep update

# 5) Create non-root user and groups safely, grant sudo (no password)
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} && \
    # ensure groups exist before adding user to them
    groupadd -f video && \
    groupadd -f render && \
    usermod -aG video,render ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME} && \
    chmod 0440 /etc/sudoers.d/90-${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# 6) Auto-source ROS in shell
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

# 7) Default entrypoint
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["bash"]
