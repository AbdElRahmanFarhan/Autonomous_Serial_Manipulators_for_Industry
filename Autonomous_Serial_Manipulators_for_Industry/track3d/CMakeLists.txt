cmake_minimum_required(VERSION 2.8.3)
project(track3d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV 3.4.2 REQUIRED  PATHS "/usr/local" NO_DEFAULT_PATH)
#link_directories(${OpenCV_LIB_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
	#OpenCV
	roscpp
	rospy
	std_msgs
	message_generation
	message_runtime
#	cv_bridge
#	image_transport
)


find_package(OpenGL)
if(NOT OPENGL_FOUND)
	  message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
endif()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

catkin_python_setup()

add_message_files(
	DIRECTORY msg FILES ball_trajectory.msg img_stream.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
#	sensor_msgs
)
catkin_package(
	CATKIN_DEPENDS message_runtime
	      std_msgs
	roscpp
	rospy
#	cv_bridge
#	      image_transport
        )

set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

if(WIN32)
	  list(APPEND DEPENDENCIES glfw3)
else()
	  # Find glfw header
		find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
			  PATHS /usr/X11R6/include
				      /usr/include/X11
							/opt/graphics/OpenGL/include
							/opt/graphics/OpenGL/contrib/libglfw
							/usr/local/include
							/usr/include/GL
							/usr/include
							)
						# Find glfw library
		find_library(GLFW_LIBRARIES NAMES glfw glfw3
			      PATHS /usr/lib64
						      /usr/lib
									/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
									/usr/local/lib64
									/usr/local/lib
									/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
									/usr/X11R6/lib
									)
endif()

find_package(realsense2)
if(NOT realsense2_FOUND)
	  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

add_executable(tracking src/tracking.cpp)
add_dependencies(tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	realsense2
	${OPENGL_LIBRARIES}
	${OpenCV_LIBS}
	glfw
)
add_executable(fetch_and_align src/fetch_and_align.cpp)
add_dependencies(fetch_and_align ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fetch_and_align
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	${OpenCV_LIBS}
	realsense2
	${OPENGL_LIBRARIES}
	glfw
)
add_executable(ref_assign src/ref_assign.cpp)
add_dependencies(ref_assign ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_assign
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	realsense2
	${OPENGL_LIBRARIES}
	${OpenCV_LIBS}
	glfw
)
add_executable(fetch_and_align_assign src/fetch_and_align_assign.cpp)
add_dependencies(fetch_and_align_assign ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fetch_and_align_assign
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	${OpenCV_LIBS}
	realsense2
	${OPENGL_LIBRARIES}
	glfw
)
add_executable(cam_accuracy src/cam_accuracy.cpp)
add_dependencies(cam_accuracy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_accuracy
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	realsense2
	${OPENGL_LIBRARIES}
	${OpenCV_LIBS}
	glfw
)

add_executable(line_detection src/line_detection.cpp)
add_dependencies(line_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_detection
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	realsense2
	${OPENGL_LIBRARIES}
	${OpenCV_LIBS}
	glfw
)

add_executable(traj_read src/traj_read.cpp)
add_dependencies(traj_read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_read
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	${OpenCV_LIBS}
	realsense2
	${OPENGL_LIBRARIES}
	glfw
)
install(PROGRAMS scripts/process_and_predict.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

