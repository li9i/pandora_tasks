cmake_minimum_required(VERSION 2.8.3)
project(simple_robot_msgs)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

## Add messages
add_message_files(DIRECTORY msg FILES TemperatureReading.msg VictimFound.msg)

## Add actions
add_action_files(DIRECTORY action FILES GetRobotPose.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS roscpp actionlib message_runtime
  DEPENDS Boost
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
