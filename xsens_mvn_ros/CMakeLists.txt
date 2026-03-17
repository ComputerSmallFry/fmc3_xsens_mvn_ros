cmake_minimum_required(VERSION 3.16)
project(xsens_mvn_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(xsens_mvn_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    xsens_mvn_sdk/include
    ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(xsens_mvn_sdk)

# HumanDataHandler library
add_library(${PROJECT_NAME}_HumanDataHandler SHARED
  src/xsens_client/HumanDataHandler.cpp
)
target_include_directories(${PROJECT_NAME}_HumanDataHandler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_HumanDataHandler
  Eigen3
)

# XSensClient library
add_library(${PROJECT_NAME}_XSensClient SHARED
  src/xsens_client/XSensClient.cpp
  src/xsens_client/Socket.cpp
)
target_include_directories(${PROJECT_NAME}_XSensClient PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_XSensClient
  ${PROJECT_NAME}_HumanDataHandler
  ${PROJECT_NAME}_SDK
)
ament_target_dependencies(${PROJECT_NAME}_XSensClient
  Eigen3
)

# Main executable
add_executable(xsens_client src/xsens_client_node.cpp)
target_link_libraries(xsens_client
  ${PROJECT_NAME}_XSensClient
)
ament_target_dependencies(xsens_client
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  xsens_mvn_ros_msgs
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install targets
install(TARGETS
  xsens_client
  ${PROJECT_NAME}_XSensClient
  ${PROJECT_NAME}_HumanDataHandler
  ${PROJECT_NAME}_SDK
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch and rviz files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
