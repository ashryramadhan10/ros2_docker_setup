#!/bin/bash
# Modular ROS2 Development Environment Setup

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLERS_DIR="$SCRIPT_DIR/installers"

# Source installer modules
source "$INSTALLERS_DIR/onnx_runtime.sh"
source "$INSTALLERS_DIR/opencv.sh"
source "$INSTALLERS_DIR/ros2sysmon.sh"

# Parse command line arguments
INSTALL_ALL=true
INSTALL_ONNX=false
INSTALL_OPENCV=false
INSTALL_ROS2SYSMON=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --onnx)
            INSTALL_ALL=false
            INSTALL_ONNX=true
            shift
            ;;
        --opencv)
            INSTALL_ALL=false
            INSTALL_OPENCV=true
            shift
            ;;
        --ros2sysmon)
            INSTALL_ALL=false
            INSTALL_ROS2SYSMON=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --onnx        Install only ONNX Runtime"
            echo "  --opencv      Install only OpenCV"
            echo "  --ros2sysmon  Install only ROS2 System Monitor"
            echo "  --help        Show this help"
            echo ""
            echo "Without options, installs all components"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "=== ROS2 Development Environment Setup ==="

# Install components based on arguments
if [[ "$INSTALL_ALL" == true ]]; then
    echo "Installing all components..."
    install_onnx_runtime
    install_opencv
    install_ros2sysmon
else
    [[ "$INSTALL_ONNX" == true ]] && install_onnx_runtime
    [[ "$INSTALL_OPENCV" == true ]] && install_opencv
    [[ "$INSTALL_ROS2SYSMON" == true ]] && install_ros2sysmon
fi

# Source workspace if ros2sysmon was installed
if [[ "$INSTALL_ALL" == true ]] || [[ "$INSTALL_ROS2SYSMON" == true ]]; then
    cd /workspace
    echo "Sourcing workspace..."
    source install/setup.bash
fi

# Test installations
echo ""
echo "=== Testing Installations ==="

# Test ONNX Runtime
if [[ "$INSTALL_ALL" == true ]] || [[ "$INSTALL_ONNX" == true ]]; then
    echo -n "ONNX Runtime: "
    if cmake --find-package -DNAME=onnxruntime -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST >/dev/null 2>&1; then
        echo "✓ OK"
    else
        echo "✗ FAILED"
    fi
fi

# Test OpenCV
if [[ "$INSTALL_ALL" == true ]] || [[ "$INSTALL_OPENCV" == true ]]; then
    echo -n "OpenCV: "
    if cmake --find-package -DNAME=OpenCV -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST >/dev/null 2>&1; then
        echo "✓ OK"
    else
        echo "✗ FAILED"
    fi
fi

echo ""
echo "=== Setup Complete ==="