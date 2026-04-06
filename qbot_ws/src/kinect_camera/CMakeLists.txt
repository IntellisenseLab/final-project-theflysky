cmake_minimum_required(VERSION 3.8)
project(kinect_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(LIBFREENECT_ROOT "$ENV{LIBFREENECT_ROOT}" CACHE PATH
  "Path to a libfreenect install prefix or source checkout")

set(_libfreenect_hints)
if(LIBFREENECT_ROOT)
  list(APPEND _libfreenect_hints "${LIBFREENECT_ROOT}")
endif()
list(APPEND _libfreenect_hints /usr /usr/local /opt/local /opt/homebrew)

find_path(LIBFREENECT_INCLUDE_DIR
  NAMES libfreenect.h
  HINTS ${_libfreenect_hints}
  PATH_SUFFIXES include include/libfreenect)

find_library(LIBFREENECT_LIBRARY
  NAMES freenect
  HINTS ${_libfreenect_hints}
  PATH_SUFFIXES lib lib64 build/lib)

if(NOT LIBFREENECT_INCLUDE_DIR OR NOT LIBFREENECT_LIBRARY)
  message(FATAL_ERROR
    "libfreenect was not found. Install Ubuntu's 'freenect' package, or set "
    "LIBFREENECT_ROOT to a libfreenect install/source root before running "
    "colcon build.")
endif()

add_executable(kinect_rgbd_node src/kinect_rgbd_node.cpp)
target_compile_features(kinect_rgbd_node PUBLIC cxx_std_17)
target_include_directories(kinect_rgbd_node PRIVATE
  ${LIBFREENECT_INCLUDE_DIR})
ament_target_dependencies(kinect_rgbd_node rclcpp sensor_msgs)
target_link_libraries(kinect_rgbd_node ${LIBFREENECT_LIBRARY})

install(TARGETS kinect_rgbd_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
