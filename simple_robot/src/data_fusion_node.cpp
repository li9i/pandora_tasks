#include "ros/ros.h"
#include "simple_robot/TemperatureReading.h"
#include "simple_robot/VictimFound.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "data_fusion_node.h"

DataFusionNode::DataFusionNode(ros::NodeHandle n, int threshold) : n_(n), threshold_(threshold) {
  
  sub_ = n_.subscribe("sensors/temperature", 1000, &DataFusionNode::data_fusion_callback, this);  
  ROS_INFO("Data fusion node subscribed to sensors/temperature topic");
  chatter_pub_  = n_.advertise<simple_robot::VictimFound> ("data_fusion/victim_found", 1000); 
  
}


void DataFusionNode::data_fusion_callback(const simple_robot::TemperatureReading::ConstPtr& temperature_msg) {
  
  if (temperature_msg->temperature >= threshold_) {
    ROS_INFO("Temperature threshold (%d) exceeded. Temp = %d", threshold_, temperature_msg->temperature);
    ROS_INFO("Publishing message [victim found] via %s device", vic_found.source.c_str());
    chatter_pub_.publish(vic_found);
  }
  
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "data_fusion_node");

  ros::NodeHandle n;
  
  int threshold = 0;
  ROS_INFO("Initiating data fusion node. Threshold = %d", threshold);
  DataFusionNode df(n, threshold);
  
  ros::spin();

  return 0;
}
