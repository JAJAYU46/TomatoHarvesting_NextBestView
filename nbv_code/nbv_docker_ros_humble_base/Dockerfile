# Start with ROS 2 Humble base image
FROM ros:humble-ros-base

# Set environment variables (for non-interactive installs)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei  
# Change to your timezone

# Install system dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    build-essential \
    cmake \
    git \
    wget \
    libtbb-dev \
    ccache \
    lsof \
    usbutils \
    && rm -rf /var/lib/apt/lists/*
# lsof & usbutils are just good packages for looking usb port connection...

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-octomap* \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*
# Set up environment for ROS 2
ENV ROS_DISTRO=humble
# ENV ROS2_WS=/home/ros2_ws2

# Install Python packages
RUN pip install --no-cache-dir \
    deep_sort_realtime \
    ultralytics \
    catkin_pkg \
    rosdistro \
    rospkg\
    pynput\
    open3d

# for [my_robot_nbv]  nbv_tom_detect: because numpy2.x & 1.x is not compatible with scipy1.15...
# we need to downgrade numpy and scipy
RUN pip uninstall numpy scipy -y && \
    pip install "numpy<2" "scipy<1.12"


# Note:
# [for my_robot_cam package ]
# cloud_rate_transformer: pynput
# [my_robot_nbv] 
# nbv_tom_detect: open3d, deep_sort_realtime, ultralytics




# # Install Intel RealSense SDK
# RUN apt-get update && apt-get install -y \
#     librealsense2-dev \
#     librealsense2-utils \
#     librealsense2-dkms \
#     && ldconfig

RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add Intel RealSense repository (Replacing apt-key adv)
RUN curl -sSL https://librealsense.intel.com/Debian/librealsense.pgp | gpg --dearmour -o /usr/share/keyrings/librealsense-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/librealsense-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update

# Install RealSense SDK
RUN apt-get install -y \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# Things for realsense ros2 package
RUN apt-get update && apt-get install -y ros-humble-cv-bridge && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-humble-diagnostic-updater && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /home/ros2_ws2

# Source ROS 2 environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command: start bash
CMD ["/bin/bash"]
