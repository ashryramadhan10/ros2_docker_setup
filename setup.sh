#!/bin/bash

# ================= ONNX Runtime =================
# Install ONNX Runtime if not already installed
if [ ! -f "/usr/local/lib/libonnxruntime.so" ]; then
    echo "Installing ONNX Runtime 1.22.1..."
    
    # Install required tools
    apt-get update && apt-get install -y wget unzip
    
    TEMP_DIR="/tmp/onnxInstall"
    mkdir -p $TEMP_DIR
    cd $TEMP_DIR
    
    wget -O onnx_archive.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.22.1
    unzip onnx_archive.nupkg
    
    cd runtimes/linux-x64/native/
    ln -s libonnxruntime.so libonnxruntime.so.1.22.1
    cp libonnxruntime.so /usr/local/lib/
    cp libonnxruntime.so.1.22.1 /usr/local/lib/
    
    cd $TEMP_DIR
    mkdir -p /usr/local/include/onnxruntime/
    cp -r build/native/include/ /usr/local/include/onnxruntime/
    
    mkdir -p /usr/local/share/cmake/onnxruntime/
    
    tee /usr/local/share/cmake/onnxruntime/onnxruntimeVersion.cmake > /dev/null << 'EOF'
set(PACKAGE_VERSION "1.22.1")
if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
EOF
    
    tee /usr/local/share/cmake/onnxruntime/onnxruntimeConfig.cmake > /dev/null << 'EOF'
include(FindPackageHandleStandardArgs)
get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime/include)
set(onnxruntime_LIBRARIES onnxruntime)
set(onnxruntime_CXX_FLAGS "")
find_library(onnxruntime_LIBRARY onnxruntime PATHS "${onnxruntime_INSTALL_PREFIX}/lib")
add_library(onnxruntime SHARED IMPORTED)
set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")
find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)
EOF
    
    ldconfig
    rm -rf $TEMP_DIR
    echo "ONNX Runtime installation completed!"
else
    echo "ONNX Runtime already installed"
fi

# ================= ONNX Runtime =================

# ================= OpenCV =================
# Install OpenCV if not already installed
if ! pkg-config --exists opencv4; then
    echo "Installing OpenCV..."
    apt-get update && apt-get install -y \
        libopencv-dev \
        python3-opencv
    echo "OpenCV installation completed!"
else
    echo "OpenCV already installed"
fi
# ================= OpenCV =================

# Change to workspace directory
cd /workspace

# Create src directory if it doesn't exist
mkdir -p src

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# for ros2 ros2sysmon
python3 -m pip install textual

# Clone ros2sysmon if it doesn't exist
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

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Testing all installation in cmake

# Test ONNX Runtime
cmake --find-package -DNAME=onnxruntime -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST

# Test OpenCV
cmake --find-package -DNAME=OpenCV -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST