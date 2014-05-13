#include "ros/ros.h"
#include <iostream>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <simple_robot/GetRobotPoseAction.h>
#include "simple_robot/VictimFound.h"
#include "victim_announce_node.h"

VictimAnnounceClient::VictimAnnounceClient(ros::NodeHandle n) : n_(n) {
  
  ac_ = new va_action_client("slam/get_robot_pose", true);
  ROS_INFO("Victim announce node is waiting for action server to start.");
  ac_->waitForServer(); 
  ROS_INFO("Action server started"); 

  sub_ = n_.subscribe("data_fusion/victim_found", 1000, &VictimAnnounceClient::va_callback, this);  
  ROS_INFO("Victim announce node subscribed to data_fusion/victim_found"); 
 
}


void VictimAnnounceClient::va_callback(const simple_robot::VictimFound::ConstPtr &victim_found_msg) {

  ac_->sendGoal(goal_);
  bool finished_before_timeout = ac_->waitForResult();
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_->getState();
    ROS_INFO("Action of RobotPoseServer finished: %s", state.toString().c_str());
    const simple_robot::GetRobotPoseResult::ConstPtr& result = ac_->getResult();
    ROS_INFO("Victim found! Robot Pose = (%d,%d). Sensor used for identification = %s", result->x, result->y, victim_found_msg->source.c_str());
    
  }    
  
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "victim_announce_node");

  ros::NodeHandle n;
  

  VictimAnnounceClient va(n);
  
  ros::spin();

  return 0;
  
}
