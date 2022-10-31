cmake_minimum_required(VERSION 3.0.2)

project(tm_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch rviz meshes models worlds xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
