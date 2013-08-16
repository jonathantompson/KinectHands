project(root)
set(root_VERBOSE_MAKEFILE false)  # Prints out compiler invocation for each src file
set(root_PRINT_SOURCE_FILES false)  # Prints out the list of files cmake finds
cmake_minimum_required(VERSION 2.8.10)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# CMAKE MODULES PATH
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# SUBPROJECTS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

if(DEFINED CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES debug))
  set(BUILD debug)
else()
  set(BUILD release)
endif()
message( STATUS "Top level build type: " ${BUILD} )
if(BUILD MATCHES release)
    message( STATUS "For debug build use cmake flag -DCMAKE_BUILD_TYPE=DEBUG" )
endif()
message( STATUS "Verbose Makefile: " ${root_VERBOSE_MAKEFILE} )

set(CMAKE_VERBOSE_MAKEFILE root_VERBOSE_MAKEFILE)

# subdirectories that are NOT in the current directory tree, you must specify
# a build directory.  The build directories here will build a local copy of the
# jtil and jcl libraries (this is OK, it just wastes disk space).
add_subdirectory(../PRenderer2/jtil "${CMAKE_CURRENT_BINARY_DIR}/jtil")
add_subdirectory(../PRenderer2/jcl "${CMAKE_CURRENT_BINARY_DIR}/jcl")
add_subdirectory(jtorch)
add_subdirectory(jtorch_test)
add_subdirectory(kinect_interface)
add_subdirectory(KinectHands)
