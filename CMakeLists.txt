cmake_minimum_required(VERSION 2.8.3)
project(esr_radar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  diagnostic_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  can_msgs
)

add_message_files(
   FILES
  Object.msg
  ObjectArray.msg
 )

generate_messages(
	DEPENDENCIES
		sensor_msgs
		std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(esr_radar_node src/esr_radar.cpp )
add_dependencies(esr_radar_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_radar_node ${catkin_LIBRARIES})

 
