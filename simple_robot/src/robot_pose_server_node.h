class RobotPoseServer {
  
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<simple_robot::GetRobotPoseAction> as_; 
  std::string action_name_;
  // create message that is used to publish result
  simple_robot::GetRobotPoseResult result_;

public:

  RobotPoseServer(std::string name);
  ~RobotPoseServer(void);
    
  void executeCB(const simple_robot::GetRobotPoseGoalConstPtr &goal);
      
};
