cmake_minimum_required(VERSION 3.8)
project(apriltag_relocalizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen)
find_package(apriltag_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# === LOCAL APRILTAG (from install/apriltag) ===
set(APRILTAG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../install/apriltag)

find_library(APRILTAG_LIB
  NAMES apriltag
  PATHS ${APRILTAG_ROOT}/lib
  NO_DEFAULT_PATH
)

find_path(APRILTAG_INCLUDE
  NAMES apriltag/apriltag.h
  PATHS ${APRILTAG_ROOT}/include
  NO_DEFAULT_PATH
)

if(NOT APRILTAG_LIB)
  message(FATAL_ERROR "libapriltag.so not found! Run: colcon build --packages-select apriltag")
endif()
if(NOT APRILTAG_INCLUDE)
  message(FATAL_ERROR "apriltag/apriltag.h not found! Run: colcon build --packages-select apriltag")
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(apriltag_relocalizer_node src/apriltag_relocalizer.cpp)

ament_target_dependencies(apriltag_relocalizer_node
  rclcpp geometry_msgs sensor_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs std_srvs nav_msgs
  cv_bridge image_transport apriltag_msgs visualization_msgs ament_index_cpp
  rclcpp_action nav2_msgs ament_index_cpp
)

target_link_libraries(apriltag_relocalizer_node
  yaml-cpp
  ${OpenCV_LIBRARIES}
  ${APRILTAG_LIB}
)

install(TARGETS apriltag_relocalizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
