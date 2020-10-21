cmake_minimum_required(VERSION 2.8.3)
project(neo_usboard_v2)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
	 nav_msgs
	 roscpp
	 tf
	 std_msgs
	 sensor_msgs
 )
 
add_subdirectory(pilot-base)
add_subdirectory(pilot-usboard)

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  DIRECTORY msg
#  FILES EmergencyStopState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES std_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs tf nav_msgs sensor_msgs
    INCLUDE_DIRS generated/include
)

###########
## Build ##
###########

file(GLOB GENERATED_SOURCE_FILES generated/src/*.cpp)

include_directories(
	generated/include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(neo_usboard_v2 src/neo_usboard_v2_node.cpp ${GENERATED_SOURCE_FILES})

add_dependencies(neo_usboard_v2 ${catkin_EXPORTED_TARGETS})

target_link_libraries(neo_usboard_v2 pilot_usboard pilot_base vnx_base ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS neo_usboard_v2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
