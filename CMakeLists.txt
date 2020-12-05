cmake_minimum_required(VERSION 3.0.2)
project(franka_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES franka_plugin
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(joint_pos_motion
  include/franka_plugin/joint_pose_motion_generator.h
  src/joint_pose_motion_generator.cpp
)

add_library(franka_driver
  include/franka_plugin/franka_driver.h
  src/franka_driver.cpp 
)

target_link_libraries(franka_driver frankx joint_pos_motion ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(joint_pos_motion ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(franka_example src/example_franka_driver.cpp)
target_link_libraries(franka_example franka_driver)
