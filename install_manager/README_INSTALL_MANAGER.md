# ROS2 Dependency Manager

A modular Python-based installation system for managing ROS2 development dependencies.

## Quick Start

```bash
# List available components
./install_manager.py --list

# Interactive mode - choose what to install
./install_manager.py --interactive

# Install specific components
./install_manager.py --install onnx_runtime opencv

# Install from profile
./install_manager.py --profile full

# Check installation status
./install_manager.py --status
```

## Available Components

- **onnx_runtime**: ONNX Runtime 1.22.1 with CMake support
- **opencv**: OpenCV development libraries
- **ros2sysmon**: ROS2 system monitoring tool

## Profiles

- **minimal**: Basic setup (opencv, ros2sysmon)
- **full**: Complete environment (all components)

## Usage Examples

```bash
# Dry run to preview installation
./install_manager.py --profile full --dry-run

# Install with verbose logging
./install_manager.py --install opencv --verbose

# Uninstall components
./install_manager.py --uninstall onnx_runtime

# Update existing installations
./install_manager.py --update
```

## Configuration

Edit `install_config.yaml` to customize:
- Workspace directory
- Component versions
- Installation options

## Adding New Components

1. Create installer module in `modules/`
2. Inherit from `BaseInstaller`
3. Implement required methods:
   - `detect()`: Check if installed
   - `install()`: Installation logic
   - `verify()`: Post-install verification

Example:
```python
from core.installer import BaseInstaller

class MyComponentInstaller(BaseInstaller):
    def detect(self) -> bool:
        # Check if component exists
        return Path("/usr/local/lib/mylib.so").exists()
    
    def install(self) -> bool:
        # Installation logic
        self.run_command("apt-get install -y mypackage")
        return True
```

## Migration from setup.sh

The new system replaces your `setup.sh` with modular, maintainable components:

- ✅ Better error handling and logging
- ✅ Dependency resolution
- ✅ Selective installation
- ✅ Verification and rollback
- ✅ Configuration management
- ✅ Dry run capability