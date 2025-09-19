# Installation Guide - ROS2 Dependency Manager

## Prerequisites

- Python 3.8+
- PyYAML package: `pip install pyyaml`
- Root/sudo access for system installations

## Quick Setup

1. **Navigate to install manager**:
   ```bash
   cd install_manager
   ```

2. **Make executable**:
   ```bash
   chmod +x install_manager.py
   ```

3. **First-time setup** (Interactive mode):
   ```bash
   ./install_manager.py --interactive
   ```

## Installation Methods

### 1. Interactive Mode (Recommended)
Choose components interactively:
```bash
cd install_manager
./install_manager.py --interactive
```

### 2. Profile-based Installation
Install predefined component sets:
```bash
cd install_manager
# Minimal setup (OpenCV + ROS2SysMonitor)
./install_manager.py --profile minimal

# Full setup (All components)
./install_manager.py --profile full
```

### 3. Selective Installation
Install specific components:
```bash
cd install_manager
# Single component
./install_manager.py --install opencv

# Multiple components
./install_manager.py --install onnx_runtime opencv ros2sysmon
```

## Common Workflows

### New Environment Setup
```bash
cd install_manager
# 1. Check what's available
./install_manager.py --list

# 2. Preview full installation
./install_manager.py --profile full --dry-run

# 3. Install everything
./install_manager.py --profile full
```

### Development Workflow
```bash
cd install_manager
# Check current status
./install_manager.py --status

# Add new component
./install_manager.py --install onnx_runtime

# Update existing installations
./install_manager.py --update
```

### Troubleshooting
```bash
cd install_manager
# Verbose logging
./install_manager.py --install opencv --verbose

# Check logs
tail -f /tmp/install_manager.log

# Reinstall component
./install_manager.py --uninstall opencv
./install_manager.py --install opencv
```

## Component Details

### ONNX Runtime
- **Version**: 1.22.1
- **Installs**: Libraries, headers, CMake config
- **Location**: `/usr/local/lib/libonnxruntime.so`
- **CMake**: `find_package(onnxruntime REQUIRED)`

### OpenCV
- **Package**: System OpenCV4
- **Installs**: Development libraries + Python bindings
- **CMake**: `find_package(OpenCV REQUIRED)`

### ROS2SysMonitor
- **Source**: GitHub repository
- **Location**: `/workspace/src/ros2sysmon`
- **Usage**: `ros2 run ros2sysmon ros2sysmon`

## Configuration

Edit `install_config.yaml` to customize:

```yaml
workspace_dir: /workspace
components:
  onnx_runtime:
    version: "1.22.1"
  ros2sysmon:
    workspace_dir: /workspace
```

## Migration from setup.sh

Replace your old workflow:

**Old way**:
```bash
./setup.sh  # Install everything, no choice
```

**New way**:
```bash
cd install_manager
# Choose what you need
./install_manager.py --interactive

# Or install everything
./install_manager.py --profile full
```

## Error Handling

### Permission Errors
Run with sudo if needed:
```bash
cd install_manager
sudo ./install_manager.py --install onnx_runtime
```

### Network Issues
Check internet connection and retry:
```bash
cd install_manager
./install_manager.py --install onnx_runtime --verbose
```

### Dependency Conflicts
Uninstall and reinstall:
```bash
cd install_manager
./install_manager.py --uninstall opencv
./install_manager.py --install opencv
```

## Advanced Usage

### Custom Profiles
Create `profiles/custom.yaml`:
```yaml
name: "My Setup"
components:
  - opencv
  - onnx_runtime
```

Then use:
```bash
cd install_manager
./install_manager.py --profile custom
```

### Adding New Components
1. Create `modules/my_component.py`
2. Inherit from `BaseInstaller`
3. Implement required methods
4. Use: `cd install_manager && ./install_manager.py --install my_component`

## Verification

After installation, verify components work:
```bash
cd install_manager
# Check installation status
./install_manager.py --status

# Test CMake integration
cmake --find-package -DNAME=OpenCV -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST
cmake --find-package -DNAME=onnxruntime -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST

# Test ROS2 package
source /workspace/install/setup.bash
ros2 pkg list | grep ros2sysmon
```