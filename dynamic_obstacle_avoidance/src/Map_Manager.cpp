#include"dynamic_obstacle_avoidance/Map_Manager.hpp"

/*
Tasks Pending;
 Check and rectify errors from int getState();
*/


Map_Manager::Map_Manager(ros::NodeHandle* nh) : nh_(*nh){

  image_transport::ImageTransport it(nh_);
  img_Sub_= it.subscribe("laser/image",1,&Map_Manager::laser_img_callback,this);

  ros::spin();
}

int Map_Manager::getState(std::vector<int> img_coords){
  return img_.at<uchar>(img_coords[0],img_coords[1]);
}  // This function might be prone to errors depending upon where is it needs to be used

void Map_Manager::dispMap(std::string s){

  static int count =1;
  if(!img_.data){
    std::cerr<<"\n Error Called from dispMap() in Map_Manager \n Error obtaining image";
    return;
  }
  cv::namedWindow(s+std::to_string(count), cv::WINDOW_AUTOSIZE);
  cv::imshow(s+std::to_string(count), img_);
  cv::waitKey(0);
  count++;

}

void Map_Manager::createCfree(){
  for(int i=0;i< img_.rows;i++)
  {
    for(int j=0;j<img_.cols;j++)
   {
    if( img_.at<uchar>(i,j) >160 )
    {
      std::vector<int> free_point;
      free_point.push_back(i);
      free_point.push_back(j);
      Cfree_.push_back(free_point);
    }
   }
  }  // The two for loops can be shifted inside the callback function
}

void Map_Manager::laser_img_callback(const sensor_msgs::Image::ConstPtr& m){
  callback_count_++;

  try{
    img_ = cv_bridge::toCvShare(m,"mono_8")->image;
    cv::waitKey(30);
  }catch(cv_bridge::Exception&e)
  {
    ROS_ERROR("Couldn't convert from '%s' to 'mono8'.", m->encoding.c_str());
  }

  if(callback_count_ ==1){
    createCfree();
  }
}

int main(int argc, char** argv){
  return 0;
}
