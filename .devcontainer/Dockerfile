FROM osrf/ros:humble-desktop-full

# Install Intel RealSense SDK
RUN apt-get update && apt-get install -y software-properties-common
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update

# Install system packages first
RUN apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    gazebo \
    terminator \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    vim \
    nano \
    python3-pip \
    python3-rich \
    python3-psutil \
    python3-yaml \
    libqt5svg5-dev \
    libqt5websockets5-dev \
    libqt5x11extras5-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages separately with fresh update
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-plotjuggler-ros \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Install lazyros
RUN python3 -m pip install lazyros

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
RUN rosdep update

# Configure git safe directory
RUN git config --global --add safe.directory /workspace

WORKDIR /workspace
