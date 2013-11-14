/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include<iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ultra_complex_communication/ultra_complex_communicationAction.h>
#include "ultra_complex_communication/shutdownMsg.h"

using namespace std;


class server {
  
  ultra_complex_communication::ultra_complex_communicationGoal goal;
  
  actionlib::SimpleActionClient<ultra_complex_communication::ultra_complex_communicationAction> ac_1("player_1", true);
  actionlib::SimpleActionClient<ultra_complex_communication::ultra_complex_communicationAction> ac_2("player_2", true);
  
  ros::NodeHandle nh_;
  ros::Publisher chatter_pub;
  
  int num_moves;
  bool winner_exists;
  int winner_id;

  public:
    server              (ros::NodeHandle n_);
    void init           ();
    void show_board     ();
    
    void call_player    (const actionlib::SimpleActionClient::ConstPtr& ac, int id);
    void check_status   ();
    void check_row      (int offset, int seq);
    void check_diag     (int offset, int seq);
    void register_move  (const complex_communication::boardMsg::ConstPtr& msg);
    void proc_move      (const complex_communication::boardMsg::ConstPtr& msg);
    void game_over      ();
};





server::server(ros::NodeHandle nh) : nh_(nh) {
 
  string topic = ros::names::clean(nh_.resolveName("/task3/shutdown"));
  chatter_pub  = nh_.advertise<ultra_complex_communication::shutdownMsg> (topic, 1000, true); 
  
  winner_exists = false;
  winner_id = 0;
  num_moves = 0;
  
  for (unsigned int i = 0; i < 9; i++) {
    goal.board_array[i] = 0;
  }
  
  
  ROS_INFO("Waiting for action servers to start.");
  
  // wait for the action server to start
  ac_1.waitForServer(); 
  ac_2.waitForServer();                  

  ROS_INFO("Action servers started"); 
  
  ROS_INFO("Commencing. Server is publishing in the console the initial board setting");
  ROS_INFO("Player 1 (X) | Player 2 (O) | available (-)");
  show_board();
  
}


void server::init() {
 
  int recipient = 0;
  srand(time(NULL));
  recipient = rand() % 2 + 1;
  
  while (!winner_exists && num_moves < 10) {
    if (recipient == 1)
      call_player(ac_1, 1);
    else
      call_player(ac_2, 2);
      
    recipient = (recipient == 1 ? 2 : 1);
    
    ROS_INFO("Board setting:");
    show_board();
    
    check_status();
    
    if (winner_exists) {
        ROS_INFO("Game over. Player %d has won", winner_id);
        game_over();
    }
  
  }
  
  
  if (!winner_exists) {
    ROS_INFO("Game over. Tie.");
    game_over();
  }
  

}










void server::call_player(const actionlib::SimpleActionClient::ConstPtr& ac, int id) {
  

  ac->sendGoal(goal);
  bool finished_before_timeout = ac->waitForResult(/*ros::Duration(30.0)*/);
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_INFO("Action of player %d finished: %s", id, state.toString().c_str());
    const ultra_complex_communication::ultra_complex_communicationResult::ConstPtr& result = ac->getResult();
    ROS_INFO("Result is: %d", result->x);
    goal.board_array[result->x] = id;
    //
    
  }      
  
  num_moves++;
    
}



void server::game_over() {
  
  ROS_INFO("Final board setting:");
  show_board();
  
  ROS_INFO("Server is requesting player node shutdown");
  ultra_complex_communication::shutdownMsg new_msg;
  new_msg.status = false;
  
  chatter_pub.publish(new_msg);
  
  ros::shutdown();
  /* server (action client) shut down */
}

void server::show_board() {
  
  
  for (unsigned int i = 0; i < 7; i = i + 3) {
    ROS_INFO("%s %s %s",goal.board_array[i]     == 1 ? "X" : (goal.board_array[i]     == 2 ? "O" : "-"),
                        goal.board_array[i + 1] == 1 ? "X" : (goal.board_array[i + 1] == 2 ? "O" : "-"),
                        goal.board_array[i + 2] == 1 ? "X" : (goal.board_array[i + 2] == 2 ? "O" : "-"));
  }
  ROS_INFO("\n");
  
}

void server::check_status() {
  
  check_row (3, 1); // check horizontally
  if (!winner_exists) {
    check_row(1,3); // check vertically
    if (!winner_exists) {
      check_diag(0, 4); /* check / */
      if (!winner_exists)
        check_diag(2, 2); /* check \ */
    }
  }
}

void server::check_row(int offset, int seq) {
  
  int a, b, c;
  for (unsigned int i = 0; i < 3 * offset; i = i + offset) {
    a = goal.board_array[i];
    b = goal.board_array[i + seq];
    c = goal.board_array[i + 2 * seq];
    
    if (a == b && b == c && a != 0) {
        winner_exists = true;
        winner_id = goal.board_array[i];
    }  
  }
}

void server::check_diag(int offset, int seq) {
  
  int a, b, c;
  for (unsigned int i = offset; i < offset + 1; i = i + seq) {
    a = goal.board_array[i];
    b = goal.board_array[i + seq];
    c = goal.board_array[i + 2 * seq];
  
    if (a == b && b == c && a != 0) {
        winner_exists = true;
        winner_id = goal.board_array[i];
    } 
  } 
}















int main(int argc, char **argv) {
  
	ros::init (argc, argv, "server");
  ros::NodeHandle n;
  
  server s(n);
  
  s.init();
  
  ros::spin();

  return 0;
}




