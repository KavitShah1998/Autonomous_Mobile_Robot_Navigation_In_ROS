#pragma once

#include<cmath>
#include<cstdlib>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<math.h>
#include<opencv2/highgui/highgui.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/LaserScan.h>
#include<string>
#include<unordered_map>
#include<vector>


class laser_imaging{
public:
  laser_imaging(ros::NodeHandle*);

  ~laser_imaging();

private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_S_;
  image_transport::Publisher laser_P_;
  ros::Rate loop_rate = ros::Rate(5);

  cv::Mat img_;
  int img_h_=200;
  int img_w_=200;
  int img_center_i_ = img_h_/2;
  int img_center_j_ = img_w_/2;

  std::string pub_topic_ = "laser/image";
  std::string sub_topic_ = "/scan";

  int img_resolution_ = 50;

  double world_w_ = img_w_ / img_resolution_;
  double world_h_ = img_h_ / img_resolution_;

  std::unordered_map <int, std::vector<int>> Hash_loc;  // create a Hash table to store (i,j) location of image end points for each of the 360 rays
  std::unordered_map <int, double> Hash_angl;  // create a Hash Map of distance of image boundary to image center along each of the 360* rays
  void nodeRunner();
  void bresenham(std::vector<int>, std::vector<int>, int); // function to create lines from center point to obstacle point (as white pixel) and from obstacle point to image boundary (as black pixel)
  void resetImage(); // reset the pixels of the original image to black
  int wrapAngle(int); // convert an angle from [-180, 180] to  [0,360];
  float to_d(float);  //convert an angle from radians to degrees
  float to_rad(float); //convert an angle from degrees to radians

  void subscriber_CallBack_(const sensor_msgs::LaserScan::ConstPtr &);
  void dispHashMap();  // [a debug function] used to display the hashmaps
  void publish_img();  // to publish the image as sensor_msgs on "laser/image";
  void dispImg(std::string s="OUTPUT_WINDOW"); // to display the image of environment using opencv libraries
  void initialize_HashMap(); // used to initialize the Hash Maps
};
