#include<iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "complex_communication/boardMsg.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "player.h"

using namespace std;

player::player(ros::NodeHandle n_, int id_) : n(n_), id(id_)
{

  std::string topic = ros::names::clean(n.resolveName("/task2/board"));

  sub = n.subscribe(topic, 1000, &player::board_callback, this);

  ROS_INFO("Player %d subscribed to: %s", id, topic.c_str());

  chatter_pub = n.advertise<complex_communication::boardMsg> (topic, 1000);

}



void player::board_callback(const complex_communication::boardMsg::ConstPtr &msg)
{

  if (msg->from == 0 && msg->status == false)
  {
    ROS_INFO("Player node is shutting down");
    ros::shutdown();
  }

  else if (msg->to == id)
  {
    proc_msg(msg);
  }

}



void player::proc_msg (const complex_communication::boardMsg::ConstPtr &msg)
{
  //ROS_INFO("Player %d got msg from %d", id, msg->from);
  ROS_INFO("Player %d is considering his move", id);
  complex_communication::boardMsg new_msg;

  new_msg.from = id;
  new_msg.to = 0;

  int i;

  bool cell_empty = false;

  while (!cell_empty)
  {
    srand(time(NULL));
    i = rand() % 9;

    if (msg->board_array[i] == 0)
    {
      cell_empty = true;
      new_msg.x = i;
    }
  }

  ROS_INFO("Player %d 's is move is (%d,%d)",
    id, (new_msg.x - new_msg.x % 3) / 3, new_msg.x % 3);

  chatter_pub.publish(new_msg);
}



void player::calc_move (
  const complex_communication::boardMsg::ConstPtr &msg,
  complex_communication::boardMsg &new_msg) {

  int i;

  bool cell_empty = false;

  while (!cell_empty)
  {
    i = rand() % 9;

    if (msg->board_array[i] == 0)
    {
      cell_empty = true;
      new_msg.x = id;
    }
  }
}



int main(int argc, char **argv)
{
  ros::init (argc, argv, "player");
  ros::NodeHandle n;
  //ros::Rate loop_rate(1);

  player p_1(n, 1);
  player p_2(n, 2);

  //while (ros::ok()) {
  //ros::spinOnce();
  //loop_rate.sleep();
  ros::spin();
  //}

  return 0;
}
