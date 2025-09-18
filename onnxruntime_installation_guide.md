# ONNX Runtime Installation on Linux-x64

Reference: https://medium.com/@massimilianoriva96/onnxruntime-integration-with-ubuntu-and-cmake-5d7af482136a

1. Download from:
```bash
wget -O onnx_archive.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.22.1
```

2. Unzip: 
```bash
unzip onnx_archive.nupkg
```

3. Next, proceed by copying the libraries and headers to the /usr/local/lib and /usr/local/include directories, respectively.

```bash
cd runtimes/linux-x64/native/
ln -s libonnxruntime.so libonnxruntime.so.1.22.1
sudo cp libonnxruntime.so /usr/local/lib/
sudo cp libonnxruntime.so.1.22.1 /usr/local/lib/

cd /tmp/onnxInstall
sudo mkdir -p /usr/local/include/onnxruntime/
sudo cp -r build/native/include/ /usr/local/include/onnxruntime/
```

4. Now in order to use the find_package funtion of CMake, we need to create two cmake files developed by jcarius:

```bash
sudo mkdir -p /usr/local/share/cmake/onnxruntime/
sudo nano /usr/local/share/cmake/onnxruntime/onnxruntimeVersion.cmake
```

```cmake
# Custom cmake version file by jcarius

set(PACKAGE_VERSION "1.22.1")

# Check whether the requested PACKAGE_FIND_VERSION is compatible
if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
```

sudo nano /usr/local/share/cmake/onnxruntime/onnxruntimeConfig.cmake

```cmake
# Custom cmake config file by jcarius to enable find_package(onnxruntime) without modifying LIBRARY_PATH and LD_LIBRARY_PATH
#
# This will define the following variables:
#   onnxruntime_FOUND        -- True if the system has the onnxruntime library
#   onnxruntime_INCLUDE_DIRS -- The include directories for onnxruntime
#   onnxruntime_LIBRARIES    -- Libraries to link against
#   onnxruntime_CXX_FLAGS    -- Additional (required) compiler flags

include(FindPackageHandleStandardArgs)

# Assume we are in <install-prefix>/share/cmake/onnxruntime/onnxruntimeConfig.cmake
get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime/include)
set(onnxruntime_LIBRARIES onnxruntime)
set(onnxruntime_CXX_FLAGS "") # no flags needed


find_library(onnxruntime_LIBRARY onnxruntime
    PATHS "${onnxruntime_INSTALL_PREFIX}/lib"
)

add_library(onnxruntime SHARED IMPORTED)
set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")

find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)
```

5. The onnxruntimeConfig.cmake file creates variables that you can exploit in your CMakeLists.txt :

```cmake
find_package(onnxruntime)

include_directories(${onnxruntime_INCLUDE_DIRS}/onnxruntime/include)

add_executable(${EXE_NAME} main.cpp)

target_link_libraries(${EXE_NAME}
    ${onnxruntime_LIBRARY}
)
```

Last, to check whether it is installed on your container, use these commands:
```bash
# Check library files
ls -la /usr/local/lib/libonnxruntime*

# Check headers
ls -la /usr/local/include/onnxruntime/

# Check CMake config
ls -la /usr/local/share/cmake/onnxruntime/

# Test library loading
ldconfig -p | grep onnxruntime

```