typedef actionlib::SimpleActionClient<ultra_complex_communication::ultra_complex_communicationAction> ttt_action_client;

class server {
  
  ttt_action_client *ac_1_;
  ttt_action_client *ac_2_;
  
  ultra_complex_communication::ultra_complex_communicationGoal goal;
  
  ros::NodeHandle nh_;
  ros::Publisher chatter_pub;
  
  int num_moves;
  bool winner_exists;
  int winner_id;

  public:
    server              (ros::NodeHandle nh);
    void init           ();
    void show_board     ();
    
    void call_player    (ttt_action_client *ac, int id);
    void check_status   ();
    void check_row      (int offset, int seq);
    void check_diag     (int offset, int seq);
    void game_over      ();
};
