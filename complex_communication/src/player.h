class player {
  
  int id;
  complex_communication::boardMsg board;
  ros::NodeHandle n;
  ros::Publisher chatter_pub;
  ros::Subscriber sub;

  public:
    player              (ros::NodeHandle n_, int id);
    
    void board_callback (const complex_communication::boardMsg::ConstPtr& msg);
    void proc_msg       (const complex_communication::boardMsg::ConstPtr& msg);
    void calc_move      (const complex_communication::boardMsg::ConstPtr &msg, complex_communication::boardMsg &new_msg);
};




