class server {
  
  complex_communication::boardMsg board;
  ros::NodeHandle n;
  ros::Publisher chatter_pub;
  ros::Subscriber sub;
  
  int num_moves;
  bool winner_exists;
  int winner_id;

  public:
    server              (ros::NodeHandle n_);
    void init_board     ();
    void show_board     ();
    
    void board_callback (const complex_communication::boardMsg::ConstPtr& msg);
    void check_status   ();
    void check_row      (int offset, int seq);
    void check_diag     (int offset, int seq);
    void register_move  (const complex_communication::boardMsg::ConstPtr& msg);
    void proc_move      (const complex_communication::boardMsg::ConstPtr& msg);
    void game_over      ();
};



