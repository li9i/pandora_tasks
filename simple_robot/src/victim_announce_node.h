typedef actionlib::SimpleActionClient<simple_robot::GetRobotPoseAction> va_action_client;

class VictimAnnounceClient {
  
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  va_action_client *ac_;
  simple_robot::GetRobotPoseGoal goal_;

  public:
    VictimAnnounceClient (ros::NodeHandle nh);
    void va_callback(const simple_robot::VictimFound::ConstPtr &victim_found_msg);
    void init ();

};
