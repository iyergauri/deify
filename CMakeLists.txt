cmake_minimum_required(VERSION 2.8.3)
project(deify)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sound_play
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
add_executable(orientAndSpeak src/orientAndSpeak)
add_executable(orient src/orient)
add_executable(speak src/speak)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(orientAndSpeak ${catkin_LIBRARIES})
target_link_libraries(orient ${catkin_LIBRARIES})
target_link_libraries(speak ${catkin_LIBRARIES})
