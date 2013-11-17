class TemperatureNode {
  
    ros::NodeHandle n_;
    ros::Publisher chatter_pub_;
    simple_robot::TemperatureReading temp_msg;
    
  public:
    TemperatureNode(ros::NodeHandle n);
    void init();
    
};
