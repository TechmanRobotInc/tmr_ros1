cmake_minimum_required(VERSION 2.8.12)
project(tm_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tm_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch meshes rviz urdf xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
