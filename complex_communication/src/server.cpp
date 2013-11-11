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
#include "complex_communication/boardMsg.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "server.h"

using namespace std;








server::server(ros::NodeHandle n_) : n(n_) {
 
  std::string topic = ros::names::clean(n.resolveName("/task2/board"));
  
  sub = n.subscribe(topic, 1000, &server::board_callback, this);
  //ROS_INFO("Server subscribed to: %s", topic.c_str());
  
  chatter_pub  = n.advertise<complex_communication::boardMsg> (topic, 1000, true); 
  
  winner_exists = false;
  winner_id = 0;
  
  init_board();  

}


void server::init_board() {
  
  num_moves = 0;
  
  for (unsigned int i = 0; i < 9; i++) {
    board.board_array[i] = 0;
  }
  
  board.status = true;  
  board.from = 0;
  board.to = rand() % 2 + 1; 
  
  ROS_INFO("Commencing. Server is publishing the initial board setting");
  ROS_INFO("Player 1 (X) | Player 2 (O)");
  ROS_INFO("Player %d starts playing", board.to);

  chatter_pub.publish(board);
}



void server::board_callback(const complex_communication::boardMsg::ConstPtr& msg) {

  if (msg->from != 0 && msg->to == 0) {
    
      num_moves++;
      //ROS_INFO("Server is processing move %d", num_moves);
      
      if (num_moves < 9)
        proc_move(msg);
  }
  
  if (num_moves == 9) {
    
    register_move(msg);
    
    if (!winner_exists) 
        ROS_INFO("Game over. Tie.");
    
    game_over();
  }
  
}



void server::proc_move (const complex_communication::boardMsg::ConstPtr& msg) {
  
  
  register_move(msg);


  complex_communication::boardMsg new_msg;
  
  for (unsigned int i = 0; i < 9; i++) {
      new_msg.board_array[i] = board.board_array[i];
  }
  
  new_msg.status = true;
  new_msg.from = 0;
  new_msg.to = msg->from == 1 ? 2 : 1;
  
  /* alt
  ros::Rate poll_rate(100);
  while(chatter_pub.getNumSubscribers() < 2) 
      poll_rate.sleep();
  * */
      
  chatter_pub.publish(new_msg);
  
}


void server::game_over() {
  
  ROS_INFO("Final board setting:");
  show_board();
  
  ROS_INFO("Server is requesting player node shutdown");
  complex_communication::boardMsg new_msg;
  new_msg.status = false;
  new_msg.from = 0;
  
  chatter_pub.publish(new_msg);
  
  ros::shutdown();
}

void server::show_board() {
  
  
  for (unsigned int i = 0; i < 7; i = i + 3) {
    ROS_INFO("%s %s %s",board.board_array[i]     == 1 ? "X" : (board.board_array[i]     == 2 ? "O" : "-"),
                        board.board_array[i + 1] == 1 ? "X" : (board.board_array[i + 1] == 2 ? "O" : "-"),
                        board.board_array[i + 2] == 1 ? "X" : (board.board_array[i + 2] == 2 ? "O" : "-"));
  }
  ROS_INFO("\n");
  
}

void server::register_move(const complex_communication::boardMsg::ConstPtr& msg) {
  
  //ROS_INFO("server got msg from %d", msg->from);
  
  board.board_array[msg->x] = msg->from;
  
  ROS_INFO("Board setting:");
  show_board();
  
  check_status();
  
  if (winner_exists) {
      ROS_INFO("Game over. Player %d has won", winner_id);
      game_over();
  }
    
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
    a = board.board_array[i];
    b = board.board_array[i + seq];
    c = board.board_array[i + 2 * seq];
    
    if (a == b && b == c && a != 0) {
        winner_exists = true;
        winner_id = board.board_array[i];
    }  
  }
}


void server::check_diag(int offset, int seq) {
  
  int a, b, c;
  for (unsigned int i = offset; i < offset + 1; i = i + seq) {
    a = board.board_array[i];
    b = board.board_array[i + seq];
    c = board.board_array[i + 2 * seq];
  
    if (a == b && b == c && a != 0) {
        winner_exists = true;
        winner_id = board.board_array[i];
    } 
  } 
}





int main(int argc, char **argv) {
  
	ros::init (argc, argv, "server");
  ros::NodeHandle n;
  
  
  //ros::Rate loop_rate(1);
  
  server s(n);
  
  //while (ros::ok() and s.get_status() == true) {
    //ros::spinOnce();
    //loop_rate.sleep();
    ros::spin();
//}
  
  return 0;
}





