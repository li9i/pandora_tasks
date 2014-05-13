#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ultra_complex_communication/ultra_complex_communicationAction.h>
#include "ultra_complex_communication/shutdownMsg.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ultra_complex_communication");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ultra_complex_communication::ultra_complex_communicationAction> ac_1("player_1", true);
  actionlib::SimpleActionClient<ultra_complex_communication::ultra_complex_communicationAction> ac_2("player_2", true);

  ROS_INFO("Waiting for action servers to start.");
  // wait for the action server to start
  ac_1.waitForServer(); //will wait for infinite time
  ac_2.waitForServer();                  // kserei poios einai o server

  ROS_INFO("Action servers started, sending goal.");
  // send a goal to the action
  ultra_complex_communication::ultra_complex_communicationGoal goal;
  
  
  
  for (unsigned int i = 0; i < 9; i++) {
    goal.board_array[i] = 0;
  }
  

  int recipient = 0;
  recipient = rand() % 2 + 1; 
  
  bool winner_exists = false;
  bool winner_id = 0;
  
  int num_moves = 0;
  
  
  
  
  
  
  while (/*!winner_exists*/num_moves < 10) {
    if (recipient == 1) {
      ac_1.sendGoal(goal);
      bool finished_before_timeout_1 = ac_1.waitForResult(ros::Duration(30.0));
      if (finished_before_timeout_1)
      {
        actionlib::SimpleClientGoalState state = ac_1.getState();
        ROS_INFO("Action of player 1 finished: %s",state.toString().c_str());
        const ultra_complex_communication::ultra_complex_communicationResult::ConstPtr& result = ac_1.getResult();
        ROS_INFO("Result is: %d", result->x);
        goal.board_array[result->x] = 1;
        recipient = 2;
        
      }
      else
        ROS_INFO("Action did not finish before the time out.");
    }
    else if (recipient == 2) {
      ac_2.sendGoal(goal);
      bool finished_before_timeout_2 = ac_2.waitForResult(ros::Duration(30.0));
      if (finished_before_timeout_2)
      {
        actionlib::SimpleClientGoalState state = ac_2.getState();
        ROS_INFO("Action of player 2 finished: %s",state.toString().c_str());
        const ultra_complex_communication::ultra_complex_communicationResult::ConstPtr& result = ac_2.getResult();
        ROS_INFO("Result is: %d", result->x);
        goal.board_array[result->x] = 2;
        recipient = 1;
      }
      else
        ROS_INFO("Action did not finish before the time out.");
    
    }
    

    num_moves++;
 /*   
  for (unsigned int i = 0; i < 9; i++) {
    ROS_INFO("%d | ", goal.board_array[i]);
  }
    
    for (unsigned int i = 0; i < 7; i = i + 3) {
      ROS_INFO("%s %s %s",goal.board_array[i]     == 1 ? "X" : (goal.board_array[i]     == 2 ? "O" : "-"),
                          goal.board_array[i + 1] == 1 ? "X" : (goal.board_array[i + 1] == 2 ? "O" : "-"),
                          goal.board_array[i + 2] == 1 ? "X" : (goal.board_array[i + 2] == 2 ? "O" : "-"));
    }
    ROS_INFO("\n");
 */   
  }
  

  //exit
  return 0;
}
