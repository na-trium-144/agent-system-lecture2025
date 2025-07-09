cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_tutorial)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  # image_transport
  choreonoid
  )
add_compile_options(-g)

# generate ros messages
add_message_files(
  FILES
  RobotObservation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  trajectory_msgs
)

# catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actionlib_tutorial
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)
