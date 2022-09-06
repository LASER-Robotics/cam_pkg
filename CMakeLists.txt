cmake_minimum_required(VERSION 3.0)
project(cam_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  cv_bridge
  image_geometry
  gazebo_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES cam_pkg
  CATKIN_DEPENDS roscpp image_transport 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Necessario pra fase quatro
add_executable(cam_pkg src/cam.cpp)
add_dependencies(cam_pkg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
find_package(PkgConfig REQUIRED)
target_link_libraries(cam_pkg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

