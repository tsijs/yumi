cmake_minimum_required(VERSION 2.8.3)
project(yumi_hw_interface)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-DROS=1)           # build using ROS libraries 
add_definitions(-DLINUXSOCKETS=1)  # build using LINUX SOCKETS libraries 

find_package(Boost REQUIRED COMPONENTS system thread QUIET)
find_package(abb_librws QUIET)
find_package(abb_libegm QUIET)

LIST(APPEND CATKIN_DEPENDS_LIST 
  yumi_msgs
  cmake_modules
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  simple_message
  std_msgs
  tf
  transmission_interface
  urdf
  rospy
#  message_generation
#  kdl_parser
)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)

#generate_dynamic_reconfigure_options(
#  cfg/rws.cfg
#)

if (abb_librws_FOUND)
  message("--   ABB RWS library found!")
  LIST(APPEND CATKIN_DEPENDS_LIST 
    abb_librws
  )
  if (abb_libegm_FOUND)
    message("--   ABB EGM library found!")
    message("--   All dependencies satisfied. The robot hardware interfaces (via EGM and industrial core sockets) will be built")
 
    LIST(APPEND CATKIN_DEPENDS_LIST 
      abb_libegm
    )

    add_definitions(-DHAVE_EGM=1)
  else()
    message("--   ABB EGM library not found")
  endif()
else()
  message("--   ABB RWS library not found")
  message("--   This package is highly dependent on abb RWS and EGM C++ libraries.")
  message("--   Only the hardware interface over industrial core sockets-RWS will be built.")
endif()


find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS_LIST})

LIST(APPEND SOURCEFILES
  src/yumi_hw/yumi_hw.cpp
  src/yumi_hw/yumi_hw_rws.cpp
  src/yumi_hw/arm_hw_egm.cpp
  src/yumi_hw/yumi_egm.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDS_LIST}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)
 
add_library(${PROJECT_NAME}
  ${SOURCEFILES}
)
add_executable(yumi_hw_ifce_node src/yumi_hw_ifce_node.cpp)
add_executable(yumi_gripper_node src/yumi_gripper_node.cpp)
add_dependencies(yumi_gripper_node ${PROJECT_NAME}_generate_messages_cpp)
 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries(yumi_hw_ifce_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
target_link_libraries(yumi_gripper_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
