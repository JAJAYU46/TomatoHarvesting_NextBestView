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

# for my_robot_nbv_cmake: Open3D cmake version building from source
# 1. Install CMake 3.24.1 (cmake version 3.24+ is needed for this Open3D building from source)
RUN cd /home && \
    wget https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1.tar.gz && \
    tar zxvf cmake-3.24.1.tar.gz && \
    cd cmake-3.24.1 && \
    sudo ./bootstrap && \
    sudo make && \
    sudo make install


# 2. Install Open3D dependencies and Open3D
RUN cd /home && \
    git clone https://github.com/isl-org/Open3D.git

RUN cd /home/Open3D && \
    sed -i 's/apt-get install/apt-get install -y/g' ./util/install_deps_ubuntu.sh
    #DEBIAN_FRONTEND=noninteractive ./util/install_deps_ubuntu.sh
# 3. Things needed to install before cmake & build Open3D
# Install system dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    libtbb-dev \
    libvtk9-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*  
    # Clean up to reduce image size

# Install required dependencies (Try)
RUN DEBIAN_FRONTEND=noninteractive apt-get update -y && \
    apt-get install -y build-essential cmake git \
    libgl1-mesa-dev libxi-dev libxmu-dev \
    python3-dev python3-numpy libtbb-dev libeigen3-dev \
    libjsoncpp-dev libpng-dev libjpeg-dev libglu1-mesa-dev \
    libglew-dev libglfw3-dev libassimp-dev libvtk9-dev \
    apt-utils xorg-dev libblas-dev liblapack-dev liblapacke-dev \
    libsdl2-dev libjpeg-dev gfortran && \
    apt-get install -y snapd && \
    snap && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
# Set environment variables
ENV PATH="/snap/bin:$PATH"


# Install required dependencies, including Clang
RUN apt-get update && apt-get install -y \
    clang-14 \
    lld-14 \
    libclang-14-dev \
    && rm -rf /var/lib/apt/lists/*
# Install required dependencies, including Clang, OpenMP, and NASM
RUN apt-get update && apt-get install -y \
    libomp-dev \
    nasm \
    libc++-14-dev \
    libc++abi-14-dev \
    && rm -rf /var/lib/apt/lists/*


# Set Clang as the default compiler
ENV CC=clang-14
ENV CXX=clang++-14


RUN apt-get install -y ccache && \
    export PATH="/usr/lib/ccache:$PATH"

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git \
    libgl1-mesa-dev libxi-dev libxmu-dev \
    python3-dev python3-numpy libtbb-dev libeigen3-dev \
    libjsoncpp-dev libpng-dev libjpeg-dev libglu1-mesa-dev \
    libglew-dev libglfw3-dev libassimp-dev libvtk9-dev \
    && rm -rf /var/lib/apt/lists/* 
RUN apt-get update && apt-get install -y \
    libvtk9-dev
ENV PATH=/usr/local/bin:$PATH 
RUN apt-get update && apt-get install -y \
    libvtk9-dev \
    python3 \
    && if [ ! -e /usr/bin/python ]; then ln -s /usr/bin/python3 /usr/bin/python; fi \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
    
# 4. Build Open3D
RUN cd /home/Open3D && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_CUDA_MODULE=OFF -DBUILD_GUI=ON ..

RUN cd /home/Open3D && \
    chmod +x ./util/install_deps_ubuntu.sh && \
    ./util/install_deps_ubuntu.sh
    # need to make the first time so that there will be files in build
RUN cd /home/Open3D/build && \
    make -j$(nproc) || true
    # Run make, allow failure

# To Avoid Conflict variable with math.h
RUN sed -i 's/M_PIf/M_PI_f/g' /home/Open3D/build/filament/src/ext_filament/libs/image/src/ImageSampler.cpp

RUN cd /home/Open3D/build && \
    make -j$(nproc) && \
    sudo make install
# Run ldconfig to update shared libraries
# RUN ldconfig



# # Install Open3D
# WORKDIR /opt
# RUN git clone --recursive https://github.com/isl-org/Open3D.git && \
#     cd Open3D && \
#     ./util/install_deps_ubuntu.sh && \
#     mkdir build && cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd ../.. && rm -rf Open3D

# Set working directory
WORKDIR /home/ros2_ws2

# Source ROS 2 environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command: start bash
CMD ["/bin/bash"]
