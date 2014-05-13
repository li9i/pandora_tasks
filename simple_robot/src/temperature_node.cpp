#include "ros/ros.h"
#include "simple_robot/TemperatureReading.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "temperature_node.h"


TemperatureNode::TemperatureNode(ros::NodeHandle n) : n_(n) { 
  chatter_pub_ = n_.advertise<simple_robot::TemperatureReading>("sensors/temperature", 1000);
}



void TemperatureNode::init() {
  
  ros::Rate loop_rate(10);
  srand(time(NULL));
  while (ros::ok()) {
    
    temp_msg.temperature = rand() % 21 + 20;
    ROS_INFO("-------------------------------------------------------------------------------");
    ROS_INFO("Temperature node reports: temp = %d", temp_msg.temperature);

    chatter_pub_.publish(temp_msg);
    
    loop_rate.sleep();
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_node");

  ros::NodeHandle n;
    
  TemperatureNode tn(n);
  tn.init();

  return 0;
}
