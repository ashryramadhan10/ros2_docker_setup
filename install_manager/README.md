# ROS2 Dependency Manager

A modular Python-based installation system for managing ROS2 development dependencies.

## Quick Start

```bash
cd install_manager

# List available components
./install_manager.py --list

# Interactive mode - choose what to install
./install_manager.py --interactive

# Install from profile
./install_manager.py --profile full
```

## Documentation

- **installation_guide.md**: Complete setup and usage guide
- **README_INSTALL_MANAGER.md**: Technical details and API reference

## Structure

```
install_manager/
├── install_manager.py      # Main CLI interface
├── core/                   # Core framework
├── modules/                # Component installers
├── profiles/               # Installation profiles
└── install_config.yaml     # Configuration
```