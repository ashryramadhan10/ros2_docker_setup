# ROS 2 Humble Development Container

A complete development environment for ROS 2 Humble with Intel RealSense support, Gazebo simulation, and GUI applications.

## Prerequisites

- Docker
- Visual Studio Code with Dev Containers extension
- X11 server (for GUI applications)

## Docker User Configuration

If you encounter "permission denied" errors when running Docker commands, configure your user to run Docker without sudo:

1. **Add user to docker group**:
   ```bash
   sudo usermod -aG docker $USER
   ```

2. **Apply group changes** (choose one):
   ```bash
   # Option 1: Log out and log back in
   # OR
   # Option 2: Apply changes to current session
   newgrp docker
   ```

3. **Verify Docker access**:
   ```bash
   docker run hello-world
   ```

4. **If issues persist, restart Docker service**:
   ```bash
   sudo systemctl restart docker
   ```

5. **If still persist, just reboot**:
   ```
    reboot
   ```

## Quick Start

1. **Prepare X11 forwarding** (run on host):
   ```bash
   export DISPLAY=:0
   xhost +local:root
   ```

2. **Open in VS Code**:
   - Open this folder in VS Code
   - When prompted, click "Reopen in Container"
   - Or use Command Palette: `Dev Containers: Reopen in Container`

3. **Build ROS 2 workspace**:
   ```bash
   cd /workspace
   colcon build
   source install/setup.bash
   ```

## Container Features

### Base Environment
- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble Desktop Full
- **Workspace**: `/workspace` (mounted from project root)

### Hardware Support
- **Intel RealSense**: SDK and utilities pre-installed
- **USB devices**: Full access via device mounting
- **GPU**: Host GPU access for Gazebo rendering

### Development Tools
- **Build system**: colcon with autocompletion
- **Editors**: vim, nano
- **Terminal**: terminator
- **Version control**: git
- **System monitoring**: ros2sysmon - terminal-based ROS2 system monitor
- **ROS utilities**: lazyros - lazy evaluation utilities for ROS2

### VS Code Extensions
- C/C++ IntelliSense
- Python support
- CMake Tools
- ROS extension

## Container Configuration

### Dockerfile
Located at `.devcontainer/Dockerfile`:
- Installs RealSense SDK without DKMS
- Configures Gazebo and ROS 2 packages
- Sets up development environment
- Initializes rosdep and creates workspace

### DevContainer Settings
Located at `.devcontainer/devcontainer.json`:
- **Network**: Host networking for ROS 2 communication
- **Display**: X11 forwarding with `QT_X11_NO_MITSHM=1`
- **Devices**: USB and hardware device access
- **Privileges**: Required for hardware interaction

## Usage

### Running ROS 2 Nodes
```bash
# Source the environment
source /opt/ros/humble/setup.bash

# Run example nodes
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### Using RealSense
```bash
# List connected devices
realsense-viewer

# Launch RealSense node
ros2 launch realsense2_camera rs_launch.py
```

### Gazebo Simulation
```bash
# Launch Gazebo
gazebo

# Or with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

### Building Packages
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with debug info
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### System Monitoring
```bash
# Launch ROS2 system monitor
ros2 run ros2sysmon ros2sysmon

# Controls:
# 1 - Topics + TF frames view
# 2 - ROS nodes + processes view
# r - Manual refresh
# q/x - Exit
```

## Troubleshooting

### GUI Applications Not Displaying
Ensure X11 forwarding is properly configured:
```bash
export DISPLAY=:0
xhost +local:root
```

### Permission Issues
The container runs as root for hardware access. If you encounter file permission issues on the host, adjust ownership:
```bash
sudo chown -R $USER:$USER /path/to/your/workspace
```

### RealSense Device Not Found
- Verify USB device is connected
- Check if device appears in container: `lsusb`
- Restart container if device was connected after startup

### Network Issues
The container uses host networking. Ensure no firewall rules block ROS 2 communication on ports 7400-7500.

## Customization

### Adding Packages
Edit `.devcontainer/Dockerfile` to install additional packages:
```dockerfile
RUN apt-get update && apt-get install -y \
    your-package-here \
    && rm -rf /var/lib/apt/lists/*
```

### VS Code Extensions
Add extensions to `.devcontainer/devcontainer.json`:
```json
"extensions": [
    "existing.extensions",
    "new.extension.id"
]
```

### Environment Variables
Add to `runArgs` in `.devcontainer/devcontainer.json`:
```json
"runArgs": [
    "--env=YOUR_VAR=value"
]
```
