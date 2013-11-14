cmake_minimum_required(VERSION 2.8.3)
project(ultra_complex_communication)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

## Add messages
add_message_files(FILES shutdownMsg.msg)

## Add actions
add_action_files(DIRECTORY action FILES ultra_complex_communication.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS roscpp actionlib message_runtime
  DEPENDS Boost
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


## Fibonacci server
add_executable(ucm_server src/ucm_server.cpp)
target_link_libraries(ucm_server ${catkin_LIBRARIES})
add_dependencies(ucm_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Fibonacci client
add_executable(ucm_client src/ucm_client.cpp)
target_link_libraries(ucm_client ${catkin_LIBRARIES})
add_dependencies(ucm_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(ucm_client ${catkin_EXPORTED_TARGETS})
endif()
