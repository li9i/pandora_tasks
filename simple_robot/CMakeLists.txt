cmake_minimum_required(VERSION 2.8.3)
project(simple_robot)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation simple_robot_msgs)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

## Add messages
add_message_files(DIRECTORY ../simple_robot_msgs/msg FILES TemperatureReading.msg VictimFound.msg)

## Add actions
add_action_files(DIRECTORY ../simple_robot_msgs/action FILES GetRobotPose.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS roscpp actionlib message_runtime
  DEPENDS Boost
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(temperature_node src/temperature_node.cpp)
target_link_libraries(temperature_node ${catkin_LIBRARIES})
add_dependencies(temperature_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(data_fusion_node src/data_fusion_node.cpp)
target_link_libraries(data_fusion_node ${catkin_LIBRARIES})
add_dependencies(data_fusion_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(victim_announce_node src/victim_announce_node.cpp)
target_link_libraries(victim_announce_node ${catkin_LIBRARIES})
add_dependencies(victim_announce_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(robot_pose_server_node src/robot_pose_server_node.cpp)
target_link_libraries(robot_pose_server_node ${catkin_LIBRARIES})
add_dependencies(robot_pose_server_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#if(catkin_EXPORTED_TARGETS)
#  add_dependencies(ucm_client ${catkin_EXPORTED_TARGETS})
#endif()
