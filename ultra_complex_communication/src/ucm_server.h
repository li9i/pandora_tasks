class ultra_complex_communicationAction
{
protected:

  int id_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ultra_complex_communication::ultra_complex_communicationAction> as_; 
  std::string action_name_;
  ros::Subscriber sub;
  // create message that is used to publish result
  ultra_complex_communication::ultra_complex_communicationResult result_;
  //shutdown msg

public:

  ultra_complex_communicationAction(string name, int id);
  ~ultra_complex_communicationAction(void);
  
  void node_shutdown(const ultra_complex_communication::shutdownMsg::ConstPtr &msg);
  
  void executeCB(const ultra_complex_communication::ultra_complex_communicationGoalConstPtr &goal);
      
};
