#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_robot/GetRobotPoseAction.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "robot_pose_server_node.h"

RobotPoseServer::RobotPoseServer(std::string name) :
  as_(nh_, name, boost::bind(&RobotPoseServer::executeCB, this, _1), false),
  action_name_(name)
{
  as_.start();
  ROS_INFO("Action server initiated");
}


RobotPoseServer::~RobotPoseServer(void){}


void RobotPoseServer::executeCB(const simple_robot::GetRobotPoseGoalConstPtr &goal) {
  
  bool success = true;

  ROS_INFO("Action server called");
  
  if (as_.isPreemptRequested() || !ros::ok()) 
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
    success = false;
  }
  
  
  int x;
  int y;

  //srand(time(NULL));
  x = rand() % 10;
  y = rand() % 10;
  result_.x = x;
  result_.y = y;
  
  if(success)
  {
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    ROS_INFO("Action server sent robot pose");
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
  
}
  



int main(int argc, char** argv) {
  srand(time(NULL));
  ros::init(argc, argv, "get_robot_pose_node");

  RobotPoseServer rps("slam/get_robot_pose");
  ros::spin();

  return 0;
  
}
