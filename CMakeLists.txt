cmake_minimum_required(VERSION 3.0.2)
project(franka_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Franka REQUIRED)
link_directories(${Franka_LIBRARY_DIRS})
add_definitions(${Franka_DEFINITIONS})

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Eigen3 REQUIRED)
link_directories(${Eigen3_LIBRARY_DIRS})
add_definitions(${Eigen3_DEFINITIONS})

add_subdirectory(frankx)

add_service_files(
  FILES
  bool_key.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES franka_plugin
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(joint_pos_motion
  include/franka_plugin/joint_pose_motion_generator.h
  src/joint_pose_motion_generator.cpp
)

target_link_libraries(joint_pos_motion ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(joint_pos_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${Franka_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS})

add_executable(franka_plugin src/franka_plugin.cpp)
target_link_libraries(franka_plugin joint_pos_motion ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(franka_node src/little_helper_demo/franka_node.cpp)
target_link_libraries(franka_node ${catkin_LIBRARIES} ${Franka_LIBRARIES})

add_executable(frankx_test src/frankx_test.cpp)
target_link_libraries(frankx_test ${catkin_LIBRARIES} frankx)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
