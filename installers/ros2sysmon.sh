#!/bin/bash
# ROS2 System Monitor installer

install_ros2sysmon() {
    cd /workspace
    mkdir -p src
    source /opt/ros/humble/setup.bash

    # Install Python dependency
    python3 -m pip install textual

    # Clone if doesn't exist
    if [ ! -d "src/ros2sysmon" ]; then
        echo "Cloning ros2sysmon..."
        cd src
        git clone https://github.com/pitosalas/ros2sysmon.git
        cd ..
    else
        echo "ros2sysmon already exists, skipping clone..."
    fi

    # Build the package
    echo "Building ros2sysmon..."
    colcon build --packages-select ros2sysmon

    echo "ros2sysmon installation completed!"
}

# Run if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_ros2sysmon
fi