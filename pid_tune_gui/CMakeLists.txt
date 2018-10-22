###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(pid_tune_gui)

#########################################
###### Loading 3rd party libraries.######
#########################################
add_definitions(-std=c++11)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "release")
ENDIF(NOT DEFINED CMAKE_BUILD_TYPE)

## Load PCL
find_package(PCL 1.7 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

## Load OpenRAVE
find_package(OpenRAVE)
if(${OpenRAVE_FOUND})
	add_definitions(-DHAS_OPENRAVE)
else(${OpenRAVE_FOUND})
	MESSAGE(WARNING "OpenRAVE NOT FOUND")
endif(${OpenRAVE_FOUND})

## Catkin
find_package(catkin REQUIRED COMPONENTS
			std_msgs
			sensor_msgs
			geometry_msgs
			uav_abstraction_layer
			image_transport
			cv_bridge
			std_srvs
			tf
			message_generation
			)

add_service_files(
  FILES
  Float32Param.srv
)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp image_transport uav_abstraction_layer cv_bridge geometry_msgs  std_srvs message_runtime
  INCLUDE_DIRS 
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)

#########################################
######			Executables		   ######
#########################################

## pid tune gui
file(GLOB_RECURSE PID_TUNE_GUI_FILES "src/*.cpp" "src/*.h" "src/*.inl" )
add_executable				(pid_tune_gui ${PID_TUNE_GUI_FILES} ${COMMON_FILES})
target_include_directories	(pid_tune_gui PUBLIC src/common)

target_include_directories	(pid_tune_gui PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries		(pid_tune_gui ${catkin_LIBRARIES})

target_link_libraries		(pid_tune_gui Qt5::Widgets)
target_link_libraries		(pid_tune_gui Qt5::PrintSupport)

