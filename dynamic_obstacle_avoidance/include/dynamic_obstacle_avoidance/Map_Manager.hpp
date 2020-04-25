#include<cv_bridge/cv_bridge.h>
#include<cstdlib>
#include<image_transport/image_transport.h>
#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<vector>
#include<string>
class Map_Manager{

public:
  Map_Manager(ros::NodeHandle*);  // Need a nodehandle as it subscribes to /map topic to receive local map (as a msg) as sensor_msgs/Image

  int getState(std::vector<int>);  // returns pixel value at a location specified in vector<int> {i,j}  // in image coordinate system
  bool checkObstacle(std::vector<int>);

  std::vector<int> getMapCoords(std::vector<double>);  // returns a vector<int> {i,j} in Map(i.e Image) Coord system
  std::vector<double> getWorldCoords(std::vector<int>);  //returns a vector<double> {x,y} in World_Coord system (world ref frame)

  void createCfree();
  std::vector<std::vector<int>> getCfree();  // returns the corresponding pixels which are free from obstacles

  void dispMap(std::string s="OUTPUT_WINDOW");  // Displays the map as an image file.

  ~Map_Manager();  // Destructor

private:
  cv::Mat img_;  // Variable to store map as an image [Local Variable]
  std::vector<std::vector<int>> Cfree_;  // Variable to store {i,j} coords of all the points lying in obstacle free (C- free) space.

  ros::NodeHandle nh_; // Nodehandle to create the subscriber;

  int callback_count_;
  image_transport::Subscriber img_Sub_;
  void laser_img_callback(const sensor_msgs::Image::ConstPtr &);

};
