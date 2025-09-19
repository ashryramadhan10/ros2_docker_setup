#!/bin/bash
# OpenCV installer

install_opencv() {
    if pkg-config --exists opencv4; then
        echo "OpenCV already installed"
        return 0
    fi

    echo "Installing OpenCV..."
    apt-get update && apt-get install -y \
        libopencv-dev \
        python3-opencv
    echo "OpenCV installation completed!"
}

# Run if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_opencv
fi