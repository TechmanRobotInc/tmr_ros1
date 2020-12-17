cmake_minimum_required(VERSION 2.8.3)
project(tm_driver)

add_definitions(-DROS_BUILD)
add_definitions(-DUSE_BOOST)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf
  tm_msgs
  hardware_interface
  #controller_manager
)

#set(BOOST_FLAG TRUE)
#if(BOOST_FLAG)
#add_definitions( -DUSE_BOOST )
#find_package(Boost REQUIRED COMPONENTS system thread chrono)
#endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

#if(BOOST_FLAG)
#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS actionlib control_msgs geometry_msgs roscpp sensor_msgs #hardware_interface controller_manager
#  DEPENDS boost #tm_hardware_interface
#)
#else()
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib control_msgs geometry_msgs roscpp sensor_msgs #hardware_interface controller_manager
  #DEPENDS tm_hardware_interface
)
#endif()

###########
## Build ##
###########

#if(NOT BOOST_FLAG)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  #CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
  #elseif(COMPILER_SUPPORTS_CXX0X)
  #  set(CMAKE_CXX_FLAGS "-std=c++0x")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
  endif()
#endif()

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(tm_driver
#   src/${PROJECT_NAME}/tm_driver.cpp
# )

# Hardware Interface
#add_library(tm_hardware_interface src/tm_hardware_interface.cpp)
#target_link_libraries(tm_hardware_interface
#  ${catkin_LIBRARIES}
#)

## Add cmake target dependencies of the library
# add_dependencies(tm_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
  src/tm_ros_node.cpp
  src/tm_ros_topic.cpp
  src/tm_ros_service.cpp
  src/tm_driver.cpp
  src/tm_command.cpp
  src/tm_sct_communication.cpp
  src/tm_svr_communication.cpp
  src/tm_communication.cpp
  src/tm_robot_state.cpp
  src/tm_packet.cpp
  src/tm_print.cpp
)
add_executable(tm_driver ${${PROJECT_NAME}_SOURCES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tm_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#if(BOOST_FLAG)
#target_link_libraries(tm_driver
#  tm_hardware_interface
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#else()
target_link_libraries(tm_driver
#  tm_hardware_interface
  ${catkin_LIBRARIES}
)
#endif()

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS tm_driver #tm_hardware_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

