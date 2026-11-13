cmake_minimum_required(VERSION 3.0.2)
project(armpi_ros1_workspace)

# Use Catkin's standard workspace entry point for the sourced ROS distribution.
set(_armpi_catkin_toplevel "/opt/ros/$ENV{ROS_DISTRO}/share/catkin/cmake/toplevel.cmake")
if(NOT EXISTS "${_armpi_catkin_toplevel}")
  message(FATAL_ERROR "Source /opt/ros/<ROS1 distro>/setup.bash before building this workspace")
endif()
include("${_armpi_catkin_toplevel}")
