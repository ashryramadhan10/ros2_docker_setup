#!/bin/bash
python3 -m pip install textual

# Create src directory if it doesn't exist
mkdir -p src

# Clone ros2sysmon if it doesn't exist
if [ ! -d "src/ros2sysmon" ]; then
    echo "Cloning ros2sysmon..."
    cd src
    git clone https://github.com/pitosalas/ros2sysmon.git
    cd ..
else
    echo "ros2sysmon already exists, skipping clone..."
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the package
echo "Building ros2sysmon..."
colcon build --packages-select ros2sysmon

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "Setup complete! You can now run: ros2 run ros2sysmon ros2sysmon"
