#include"dynamic_obstacle_avoidance/Map_Manager.hpp"

/*
Tasks Pending;
 Check and rectify errors from int getState();
*/

// Need a nodehandle as it subscribes to /map topic to receive local map (as a msg) as sensor_msgs/Image
Map_Manager::Map_Manager(ros::NodeHandle* nh) : nh_(*nh){

  image_transport::ImageTransport it(nh_);
  img_Sub_= it.subscribe("laser/image",1,&Map_Manager::laser_img_callback,this);

  ros::spin();
}

// returns pixel value at a location specified in vector<int> {i,j}  // in image coordinate system
int Map_Manager::getState(std::vector<int> img_coords){
  return img_.at<uchar>(img_coords[0],img_coords[1]);
}  // This function might be prone to errors depending upon where is it needs to be used

// Checking for obstacles 
bool Map_Manager::checkObstacle(std::vector<int> img_coords){
 if (img_.at<uchar>(img_coords[0],img_coords[1]) > 160)
  return false;
 else
  return true;
}

// returns a vector<int> {i,j} in Map(i.e Image) Coord system
std::vector<int> Map_Manager::getMapCoords(std::vector<double> world_coords){
 std::vector <int> img_coords;
 pixel_coord.push_back(100 - round(world_coords[0]*50));
 pixel_coord.push_back(100 - round(world_coords[1]*50));
 return img_coords;
}

// returns a vector<double> {x,y} in World_Coord system (world ref frame)
std::vector<double> Map_Manager::getWorldCoords(std::vector<int> img_coords); {
 std::vector <double> world_coords;
 map_coord.push_back(100 - img_coords[0]*0.02);
 map_coord.push_back(100 - img_coords[1]*0.02);
 return world_coords;
}

// Displays the map as an image file.
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

//  Creates a vector of free spaces 
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

// returns the corresponding pixels which are free from obstacles
std::vector<std::vector<int>> Map_Manager::getCfree(){
 return Cfree;
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
