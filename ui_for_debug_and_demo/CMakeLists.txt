cmake_minimum_required(VERSION 3.0.2)
project(ui_for_debug_and_demo)

add_compile_options(-std=c++14)
#add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tm_msgs
)

find_package(catkin REQUIRED)

# Qt5
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)
#find_package(Qt5 COMPONENTS Core Gui Network Concurrent Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()


###########
## Build ##
###########

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Specify additional locations of header files

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

qt5_wrap_ui(UIC_FILES src/tm_ros_driver_windows.ui)
set(HEADERS
  src/tm_ros_driver_windows.hpp
)

set( SRCS
src/main.cpp
src/ros_page.cpp
src/tm_ros_driver_windows.cpp
)

# Demo executable
add_executable(robot_ui ${SRCS} ${UIC_FILES} ${HEADERS})

add_dependencies(robot_ui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_ui
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Install libraries
install(TARGETS
robot_ui
	DESTINATION lib/${PROJECT_NAME}
)
# Install executables
install(TARGETS robot_ui #tm_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
