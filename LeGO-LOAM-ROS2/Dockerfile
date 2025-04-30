FROM ros:galactic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=galactic

# Update and install required packages
RUN apt update && apt install -y \
    wget \
    unzip \
    git \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    libboost-all-dev \
    libeigen3-dev \
    libtbb-dev \
    ros-galactic-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*


# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Install GTSAM
RUN apt-get update \
    && apt install -y software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.1 \
    && apt-get update \
    && apt install -y libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y vim tmux ros-galactic-rviz2 ros-galactic-rosbag2-storage-mcap ros-galactic-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y x11-apps x11-utils x11-xserver-utils

# Make sure the workspace src exists
RUN mkdir -p /root/dev_ws/src

# Set working directory and default shell
WORKDIR /root/dev_ws
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
