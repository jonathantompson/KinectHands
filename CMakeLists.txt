project(root)
set(CMAKE_VERBOSE_MAKEFILE on)
cmake_minimum_required(VERSION 2.8.10)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# CMAKE MODULES PATH
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# SUBPROJECTS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

set(root_VERBOSE_MAKEFILE false)

# subdirectories that are NOT in the current directory tree, you must specify
# a build directory.  The build directories here will build a local copy of the
# jtil and jcl libraries (this is OK, it just wastes disk space).
add_subdirectory(../PRenderer2/jtil "${CMAKE_CURRENT_BINARY_DIR}/jtil")
add_subdirectory(../PRenderer2/jcl "${CMAKE_CURRENT_BINARY_DIR}/jcl")
add_subdirectory(jtorch)
add_subdirectory(jtorch_test)
add_subdirectory(kinect_interface)
add_subdirectory(KinectHands)
