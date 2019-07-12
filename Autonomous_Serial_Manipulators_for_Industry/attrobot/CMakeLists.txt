cmake_minimum_required(VERSION 2.8.3)
project(attrobot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(moveit_tutorials REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (abb_librws REQUIRED)
find_package (abb_libegm REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  Eigen3
  abb_librws
  abb_libegm
  moveit_tutorials
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  reflexxes_type2

)

## Generate messages in the 'msg' folder
catkin_python_setup()

 add_message_files(
	 FILES
ball_trajectory.msg
control_params.msg
robot_trajectory.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
#   actionlib_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES attrobot
  CATKIN_DEPENDS
  #Eigen3
  abb_libegm
  abb_librws
  moveit_tutorials
  roscpp
  rospy
#  std_msgs
  actionlib
  actionlib_msgs
  message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${abb_libegm_INCLUDE_DIRS}
  ${abb_librws_INCLUDE_DIRS}
  ${reflexxes_type2_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(rws_demo src/rws_demo.cpp)
add_dependencies(rws_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rws_demo ${catkin_LIBRARIES})

add_executable(euler_to_quat src/euler_to_quat.cpp)
add_dependencies(euler_to_quat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(euler_to_quat ${catkin_LIBRARIES})

add_executable(egm_comm_demo src/egm_comm_demo.cpp)
add_dependencies(egm_comm_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(egm_comm_demo ${catkin_LIBRARIES})

add_executable(calculate_distance src/calculate_distance.cpp)
add_dependencies(calculate_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculate_distance ${catkin_LIBRARIES})

add_executable(transformation src/transformation.cpp)
add_dependencies(transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transformation ${catkin_LIBRARIES})

add_executable(reflexxes_controller src/reflexxes_controller.cpp)
add_dependencies(reflexxes_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflexxes_controller	 ${catkin_LIBRARIES})

install(PROGRAMS scripts/process_and_predict.py scripts/path_planning.py scripts/user_interface.py scripts/trajectory_planning.py scripts/control_optimization.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#add_executable(path_planning src/path_planning.cpp)
#add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(path_planning ${catkin_LIBRARIES})
#add_executable(motion_planning src/motion_planning.cpp)
#add_dependencies(motion_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(motion_planning ${catkin_LIBRARIES})

