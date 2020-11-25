cmake_minimum_required(VERSION 2.8.3)
project(ui_for_debug_and_demo)
add_compile_options(-std=c++14)

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
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

#find_package(Qt5 COMPONENTS Core Gui Network Concurrent Widgets REQUIRED)
catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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

add_executable(robot_ui ${SRCS} ${UIC_FILES} ${HEADERS})

add_dependencies(robot_ui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_ui
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

install(TARGETS
robot_ui
	DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS robot_ui #tm_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)