class DataFusionNode {
  
    ros::NodeHandle n_;
    ros::Publisher chatter_pub_;
    ros::Subscriber sub_;
    int threshold_;
    simple_robot::VictimFound vic_found;
    
  public:
    DataFusionNode(ros::NodeHandle n, int threshold);
    void data_fusion_callback (const simple_robot::TemperatureReading::ConstPtr& temperature_msg);
};
