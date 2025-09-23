cmake_minimum_required(VERSION 3.8)
project(beam_detector_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Handle debug/release library linking
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Force use of release libraries if debug ones are not available
    set_target_properties(${OpenCV_LIBS} PROPERTIES
            MAP_IMPORTED_CONFIG_DEBUG Release
    )
endif()

# Add ONNX Runtime
find_package(onnxruntime REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/onnxruntime/include/
)

# Create executable
add_executable(beam_detector_node src/beam_detector_node.cpp)

# Target dependencies
ament_target_dependencies(beam_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# Link libraries
target_link_libraries(beam_detector_node
  ${OpenCV_LIBS}
  ${onnxruntime_LIBRARIES}
)

# Install executable
install(TARGETS
  beam_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
