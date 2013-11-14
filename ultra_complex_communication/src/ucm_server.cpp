#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ultra_complex_communication/ultra_complex_communicationAction.h>
#include "ultra_complex_communication/shutdownMsg.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
using namespace std;
#include "ucm_server.h"




ultra_complex_communicationAction::ultra_complex_communicationAction(string name, int id) :
  as_(nh_, name, boost::bind(&ultra_complex_communicationAction::executeCB, this, _1), false),
  action_name_(name),
  id_(id)
  {
    as_.start();
    
    // subscribe to shutdown topic
    std::string topic = ros::names::clean(nh_.resolveName("/task3/shutdown"));
    sub = nh_.subscribe(topic, 1000, &ultra_complex_communicationAction::node_shutdown, this);
    ROS_INFO("Player %d subscribed to: %s", id, topic.c_str());
  }

ultra_complex_communicationAction::~ultra_complex_communicationAction(void){}
  
  

  
void ultra_complex_communicationAction::node_shutdown(const ultra_complex_communication::shutdownMsg::ConstPtr &msg) {
  
  if (msg->status == false) {
    ROS_INFO("Player node is shutting down");
    ros::shutdown();
  }
}
  


void ultra_complex_communicationAction::executeCB(const ultra_complex_communication::ultra_complex_communicationGoalConstPtr &goal) {
  
  
  bool success = true;

  ROS_INFO("Action server %d called", id_);
  ROS_INFO("Player %d is considering his move", id_);
  
  if (as_.isPreemptRequested() || !ros::ok()) // edo shutdown apo to ksexoristo msg
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
    success = false;
  }
  
  
  int i;

  bool cell_empty = false;
  while (!cell_empty) {
    srand(time(NULL));
    i = rand() % 9;
    if (goal->board_array[i] == 0) {
        cell_empty = true;
        result_.x = i;
    }
  }

  ROS_INFO("Player %d 's is move is (%d,%d)", id_, (result_.x - result_.x % 3) / 3, result_.x % 3);

  if(success)
  {
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
}
    















int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultra_complex_communication");

  ultra_complex_communicationAction player_1("player_1", 1);
  ultra_complex_communicationAction player_2("player_2", 2);
  ros::spin();

  return 0;
}
