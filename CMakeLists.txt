cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
#set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for Sophus
include_directories(
  ~/drone_autopilot/drone_sim_environment/src/req_packages/Sophus
  ~/drone_autopilot/drone_sim_environment/src/req_packages/Sophus/sophus
)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for ORB_SLAM3 and OpenCV
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}  # Add OpenCV include directories
)

# Link directories (optional, but if needed for custom libraries, you can add them here)
link_directories(
  include
)

# Add executables and link OpenCV and other dependencies
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(mono ${OpenCV_LIBS})  # Link OpenCV libraries

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd ${OpenCV_LIBS})  # Link OpenCV libraries

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(stereo ${OpenCV_LIBS})  # Link OpenCV libraries

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(stereo-inertial ${OpenCV_LIBS})  # Link OpenCV libraries

# Install targets
install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (uncomment if needed)
install(DIRECTORY launch config vocabulary
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
