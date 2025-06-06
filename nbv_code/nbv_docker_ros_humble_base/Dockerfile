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


# For Open3D
# For Install Open3D
# 1. Upgrade cmake to 3.24.1 (because building Open3D cmake from source need cmake 3.24+, the one we got was 3.22 (cmake --version) (by directly apt install cmake) )
RUN cd /home && \
    wget https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1.tar.gz && \
    tar zxvf cmake-3.24.1.tar.gz && \
    cd cmake-3.24.1 && \
    sudo ./bootstrap && \
    sudo make && \
    sudo make install
ENV PATH="/usr/local/bin:$PATH"
RUN apt update && apt install nano -y

# 2. clone Open3D files
RUN cd /home && \
    git clone https://github.com/isl-org/Open3D.git

# 3. Install Dependencies
RUN cd /home/Open3D && \
    sed -i 's/apt-get install/apt-get install -y/g' ./util/install_deps_ubuntu.sh && \
    ./util/install_deps_ubuntu.sh


# 4. Install required packages
RUN apt-get update && apt-get install -y \
    libtbb-dev \
    libvtk9-dev \
    libopenblas-dev \
    python3-dev \
    xorg-dev \
    libglu1-mesa-dev \
    libblas-dev \
    liblapack-dev \
    libsdl2-dev \
    libjpeg-dev \
    gfortran \
    build-essential \
    cmake \
    git \
    libgl1-mesa-dev \
    libxi-dev \
    libxmu-dev \
    python3-numpy \
    libeigen3-dev \
    libjsoncpp-dev \
    libpng-dev \
    libglew-dev \
    libglfw3-dev \
    libassimp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \  
    nasm \
    && rm -rf /var/lib/apt/lists/*

# Set up Python3 as the default Python
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN cd /home/Open3D && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_CUDA_MODULE=OFF -DBUILD_GUI=ON .. && \
    make -j$(nproc) || true
    # Run make, allow failure (building the first time to get the /ImageSampler.cpp file)
RUN cd /home/Open3D/build && \    
    sed -i 's/M_PIf/M_PI_f/g' /home/Open3D/build/filament/src/ext_filament/libs/image/src/ImageSampler.cpp && \
    make -j$(nproc)
RUN cd /home/Open3D/build && \   
    sudo make install

# Set working directory
WORKDIR /home/ros2_ws2

# Source ROS 2 environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command: start bash
CMD ["/bin/bash"]
