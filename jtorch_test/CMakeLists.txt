project(jtorch_test)
set(CMAKE_VERBOSE_MAKEFILE root_VERBOSE_MAKEFILE)
cmake_minimum_required(VERSION 2.8.10)

message("*************************************************")
message("************* JTORCH_TEST CMAKELISTS ************")
message("*************************************************")

if(CMAKE_BUILD_TYPE MATCHES DEBUG)
  message("cmake compilation is in debug mode, target will be jtorch_test") 
  set(TARGET_NAME jtorch_test)
else(CMAKE_BUILD_TYPE MATCHES DEBUG)
  message("cmake compilation is in release mode, target will be jtorch_test")
  set(TARGET_NAME jtorch_test)
endif(CMAKE_BUILD_TYPE MATCHES DEBUG)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# CMAKE MODULES PATH
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# EXTRA LIBS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
if(APPLE)
    message("Detected Apple")
    SET(EXTRA_LIBS "-framework Cocoa -framework OpenGL -framework IOKit")
elseif(MSVC)
    message( FATAL_ERROR, "Windows cmake is not supported.  Use visual studio." )
endif(APPLE)

set(LIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/MAC_OS_X)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# INCLUDE DIRECTORIES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

set(INCLUDE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../include/MAC_OS_X)

set(JTIL_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../PRenderer2/jtil/include/)
set(JCL_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../PRenderer2/jcl/include/)
set(JTORCH_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jtorch/include/)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# COMPILER FLAGS AND PREDEFINES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

# check the c and C++ compiler versions
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "gcc version >= 4.7")
else()
    message( FATAL_ERROR, "GCC version 4.7 or greater is required" )
    message( "on mac you need 'sudo port install gcc49' and add 'export CXX=/opt/local/bin/g++-mp-4.9'")
endif()

execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GXX_VERSION)
if (GXX_VERSION VERSION_GREATER 4.7 OR GXX_VERSION VERSION_EQUAL 4.7)
    message(STATUS "g++ Version >= 4.7")
else()
    message( FATAL_ERROR, "GCC version 4.7 or greater is required" )
    message( "on mac you need 'sudo port install gcc49' and add 'export CC=/opt/local/bin/gcc-mp-4.9'")
endif()

# -g adds debugging symbols
# -Wall turns on all warnings
# -Wextra turns on a lot of warnings (but not too pedantic)
add_definitions(-DGLFW_INCLUDE_GLCOREARB)
add_definitions(-DASSIMP_BUILD_BOOST_WORKAROUND)

# Use: "cmake -DCMAKE_BUILD_TYPE=Debug" for debug
if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -Wextra -Wno-ignored-qualifiers -Wno-unused-parameter -std=c++11")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -g -Wextra -Wno-ignored-qualifiers -Wno-unused-parameter")
    message(STATUS "adding debug flags")
    add_definitions(-DDEBUG)
    add_definitions(-D_DEBUG)
    add_definitions(-DBREAK_ON_EXCEPTION)
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -g -Wextra -Wno-ignored-qualifiers -Wno-unused-parameter -std=c++11 -O3 -msse -msse2")
    SET(CMAKE_CC_FLAGS "${CMAKE_CC_FLAGS_RELEASE} -fopenmp -g -Wextra -Wno-ignored-qualifiers -Wno-unused-parameter -O3 -msse -msse2")
    message(STATUS "adding release flags")
endif(CMAKE_BUILD_TYPE MATCHES DEBUG)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# JTORCH_TEST SOURCE AND HEADERS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
set(ROOT_HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/headers/")
set(ROOT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# JTORCH_TEST TARGET
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

file(GLOB_RECURSE JTORCH_TEST_CXX_SOURCE ${ROOT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE JTORCH_TEST_CC_SOURCE ${ROOT_SOURCE_DIR}/*.c)
file(GLOB_RECURSE JTORCH_TEST_HEADER ${ROOT_HEADER_DIR}/*.h)

message( "C++ SOURCE FILES:" )
message( STATUS ${JTORCH_TEST_CXX_SOURCE} )

message( "C SOURCE FILES:" )
message( STATUS ${JTORCH_TEST_CC_SOURCE} )

message( "HEADER FILES:" )
message( STATUS ${JTORCH_TEST_HEADER} )

message( "INCLUDE DIRECTORIES:" )
message( STATUS "jtil: " ${JTIL_INC_DIR} )
message( STATUS "jcl: " ${JCL_INC_DIR} )
message( STATUS "jtorch: " ${JTORCH_INC_DIR} )

include_directories(
    ${ROOT_HEADER_DIR} 
    ${JTIL_INC_DIR}
    ${JCL_INC_DIR}
    ${JTORCH_INC_DIR}
)

message( "EXTRA_LIBS to be linked in: " ${EXTRA_LIBS} )

add_executable(${PROJECT_NAME} ${JTORCH_TEST_CXX_SOURCE} ${JTORCH_TEST_CC_SOURCE} ${JTORCH_TEST_HEADER})

target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})
target_link_libraries(${PROJECT_NAME} jtil)
target_link_libraries(${PROJECT_NAME} jcl)
target_link_libraries(${PROJECT_NAME} jtorch)

