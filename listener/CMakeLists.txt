cmake_minimum_required(VERSION 2.8.3)
project(listener)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  turtlebot_msgs
  cmvision
  depth_image_proc
)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs turtlebot_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
###########
## Build ##
###########
add_executable(listener src/listener.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(listener ${catkin_LIBRARIES})
