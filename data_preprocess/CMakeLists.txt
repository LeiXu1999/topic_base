cmake_minimum_required(VERSION 3.0.2)
project(data_preprocess)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED
	roscpp
	rospy
	sensor_msgs
	nav_msgs
	geometry_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
	${catkin_INCLUDE_DIRS}
)
add_executable (data_preprocess 
		src/data_preprocess.cpp)
target_link_libraries(data_preprocess
	${catkin_LIBRARIES} ${OpenCV_LIBS}
)
