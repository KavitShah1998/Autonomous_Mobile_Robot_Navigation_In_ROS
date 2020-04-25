#include"dynamic_obstacle_avoidance/Laser_Imaging.hpp"

laser_imaging::laser_imaging(ros::NodeHandle* nh): nh_(*nh){

  laser_S_ = nh_.subscribe(sub_topic_, 10, & laser_imaging::subscriber_CallBack_, this);

  image_transport::ImageTransport it_(nh_);
  laser_P_ = it_.advertise(pub_topic_,1);

  cv::Mat define_img_(img_h_,img_w_,CV_8UC1, cv::Scalar(155));

  img_ = define_img_;  // initialize img_

  initialize_HashMap();
}

/*
~laser_imaging::laser_imaging()
{
  std::cout<<"\n Object of Laser_imaging_node destructed\n";
}
*/

void laser_imaging::bresenham(std::vector<int>start_pixel, std::vector<int> end_pixel, int obs_state){
// function to create lines from center point to obstacle point (as white pixel) and from obstacle point to image boundary (as black pixel)
  int i1,j1, i2,j2,i,j;
  float err=0,increment=0;
  int pix;
  float di,dj;
  double m;
  //std::cout<<"\n Start Point"<< start_pixel[0]<<" , "<<start_pixel[1];
  //std::cout<<"\n End Point"<< end_pixel[0]<<" , "<<end_pixel[1];

  if (obs_state !=1)  // plot a obs
  {
    pix = 0;
  }
  else
  {
    pix = 255;

  }
  err=0;


  di = (end_pixel[0]-start_pixel[0]);
  dj = (end_pixel[1] - start_pixel[1]);


  if(di!=0)
  {
    // assign P#1 to the pixel point with lesser x coordinate
    if(start_pixel[0]< end_pixel[0])
    {
      i1 = start_pixel[0];
      j1 = start_pixel[1];

      i2=end_pixel[0];
      j2=end_pixel[1];
    }
    else{
      i2 = start_pixel[0];
      j2 = start_pixel[1];

      i1 = end_pixel[0];
      j1 = end_pixel[1];
    }
    //std::cout<<"\n di = "<<di<<" \t  dj= "<<dj;
    m = dj/di;
    //std::cout<<"\n m= "<<(float)(dj/di);
    //std::cout<<"\n Start Pixel: "<< i1<<" , "<<j1;
    //std::cout<<"\n End Pixel :" << i2<<" , "<<j2;
    // now check if the slope of the line is Greater than 0, or Lesser than 0
    if(m>0){
        if(abs(m)<1){  //  0< m <1
          err = 0;
          i=i1; j=j1;

          while(i!=i2)
          {
            img_.at<uchar>(i,j) = pix;
            //std::cout<<"\n err "<<err<< "\t m"<<m;
            if(err + m <0.5 ) // dont increment j
            {
              i++;
              err= err + m;
            }
            else
            {
              i++;
              j++;
              err= err + m-1;
            }
          }
        }
        else{  //  1=<m< inf

          err= 0;
          i = i1;
          j=j1;

          while(j!=j2){
            //img_
            img_.at<uchar>(i,j) = pix;
            if(err + (1/m) < 0.5)
            {
              err = err + (1/m);
              j++;
            }
            else{
              err = err + (1/m) -1;
              j++;
              i++;
            }
          }
        }


    }
    else{  // m<= 0
      if(abs(m)<1){  // -1< m < =0
        err = 0;
        i=i1;
        j=j1;
        while(i!=i2){
          //plot
          img_.at<uchar>(i,j) = pix;

          if(err + m >-0.5){
            i++;
            err = err + m;
          }
          else
          {
            i++;
            j--;
            err = err  + m + 1;
          }
        }


      }

      else{
        err = 0;
        i=i1;
        j=j1;
        while(j!=j2){
          //plot
          img_.at<uchar>(i,j) = pix;

          if(err + (1/m) >-0.5){
            j--;
            err = err + (1/m);
          }
          else
          {
            j--;
            i++;
            err = err  + (1/m) + 1;
          }
        }


      }
    }

  }

  else
  {
    if(start_pixel[1]<end_pixel[1]){
      i1 = start_pixel[0]; j1 = start_pixel[1];
      i2 = end_pixel[0]; j2 = end_pixel[1];
    }
    else{
      i1 = end_pixel[0]; j1 = end_pixel[1];
      i2 = start_pixel[0]; j2 = start_pixel[1];
    }

    //
    i = i1;j=j1;
    while(j!=j2){
      //plot
      img_.at<uchar>(i,j) = pix;
      j++;
    }
  }

}


int laser_imaging::wrapAngle(int a){
  // convert an angle from [-180, 180] to  [0,360];
  a = (a + 360) % 360;
  return a;
}


float laser_imaging::to_d(float a){
   //convert an angle from radians to degrees
   return a*180/M_PI;
}

float laser_imaging::to_rad(float a){
  //convert an angle from degrees to radians
  return a*M_PI/180;
}

void laser_imaging::subscriber_CallBack_(const sensor_msgs::LaserScan::ConstPtr &m){
// this callback function converts the scan data received from each callback into opencv image
  std::cout<<"\n Recving Call backs";
   int sz = m->ranges.size();
    //int ii=30;
    for(int ii=0;ii<sz;ii++){

      int img_i_ , img_i_end;
      int img_j_, img_j_end;

      std::vector<int> robot_posit_;
      robot_posit_.emplace_back(img_center_i_);
      robot_posit_.emplace_back(img_center_j_);

      double x,y;

      //int zi=270;

      double r = m->ranges[ii];
      double t = (m->angle_min + ii*m->angle_increment);  // theta in radians

      std::cout<<"\n r= "<<r;
      std::cout<<"\n t= "<<t;
      x = r * cos(t);
      y = r * sin(t);

      std::cout<<"\n x= "<<x;
      std::cout<<"\n y= "<<y;

      std::vector<int> obstacle_pixel;
      obstacle_pixel.emplace_back(img_center_i_ - x * img_resolution_);
      obstacle_pixel.emplace_back(img_center_j_ - y* img_resolution_);

      img_i_end = (Hash_loc[ii])[0];
      img_j_end = (Hash_loc[ii])[1];
      std::vector<int> end_of_scan_line_;
      end_of_scan_line_.emplace_back(img_i_end);
      end_of_scan_line_.emplace_back(img_j_end);

      double dbl = sqrt (pow(x ,2) + pow(y,2));

      std::cout<<"\n HI  -- CallBack";
      if(dbl< Hash_angl[ii]){  // dist of obstc in this scan is within the allowable distance limit in theata direction
        bresenham( robot_posit_, obstacle_pixel,1);
        bresenham(obstacle_pixel, end_of_scan_line_,0);
        std::cout<<"\n Robot Pixel: "<< robot_posit_[0]<<" , "<<robot_posit_[1];
        std::cout<<"\n Obstacle Pixel :" << obstacle_pixel[0]<<" , "<<obstacle_pixel[1];
        std::cout<<"\n End Pixel :" << end_of_scan_line_[0]<<" , "<<end_of_scan_line_[1];
      }
      else// dist of obstc in this scan line is away from the allowable scan limit in theta direction
      {
        bresenham(robot_posit_,end_of_scan_line_,1);
        std::cout<<"\n Robot Pixel: "<< robot_posit_[0]<<" , "<<robot_posit_[1];
        std::cout<<"\n End Pixel :" << end_of_scan_line_[0]<<" , "<<end_of_scan_line_[1];
      }

    }
    std::cout<<"\n Hi there";
    dispImg();
    //std::cout<<"\n Range @ Theta  "<< (p->angle_min + ii * p->angle_increment)*180/M_PI <<" = "<< p->ranges[ii];
    //std::cout<<"\n Hello World "<<ros::Time::now();

    publish_img();


    cv::Mat define_img_(img_h_,img_w_,CV_8UC1, cv::Scalar(155));

    img_ = define_img_;
}

void laser_imaging::dispHashMap(){
// a debug function

  int  n = Hash_angl.size();
  static int count = 0;
  if(count==0)
  {
    for(int i =0; i<n;i++){
      std::cout<<"\n "<< i << " : ("<<(Hash_loc[i])[0] <<","<<(Hash_loc[i])[1]<<")" ;
    }
  }
  count++;

}

void laser_imaging::publish_img(){
  //to publish the image as a sensor_msgs on "laser/image";
  ros::Rate loop_rate(5);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_).toImageMsg();
  laser_P_.publish(msg);
  loop_rate.sleep();
}


void laser_imaging::dispImg(std::string s){
// to display the image of environment using opencv libraries

  if(!img_.data){
    std::cout<<"\n ERROR FROM $laser_imaging_node$ : Could not load the map image";
  }
  cv::namedWindow(s , cv::WINDOW_AUTOSIZE);
  cv::imshow(s, img_);
  cv::waitKey(0);
}

void laser_imaging::initialize_HashMap(){
// used to initialize the Hash Maps
  int A = (atan2(world_w_/2,world_h_/2)) * 180/M_PI;
  int B = (atan2(world_w_/2,-world_h_/2))* 180/M_PI;
  int C = (atan2(-world_w_/2,-world_h_/2))*180/M_PI;
  int D = (atan2(-world_w_/2,world_h_/2)) *180/M_PI;

  //  std::cout<<"\n a = "<<wrapAngle(A);
  //  std::cout<<"\n b = "<<wrapAngle(B);
  //  std::cout<<"\n c = "<<wrapAngle(C);
  //  std::cout<<"\n d = "<<wrapAngle(D);

  int a=wrapAngle(A);  //defining img's top left corner angle from center
  int b=wrapAngle(B);  //defining img's botton left corner angle from center
  int c=wrapAngle(C);  //defining img's bottom right corner angle from center
  int d= wrapAngle(D);  //defining img's top right corner angle from center

  int theta;
  float theta_r;
  float delta;

  int img_i_ = 0;   // i direction image index variable
  int img_j_ = 0;  // j direction image index variable
  for(int i=0;i<360;i++)
  {
    theta = i;
    theta_r = to_rad(theta);
    //std::cout<<"\n "<<theta;

    if(theta<a)
    {
      //std::cout<<"\n Theta less than 45";
      //std::cout<<"\n Case A-Da";
      delta = (img_h_/2) * tan(theta_r);
      //std::cout<<"\n "<<theta<<" \t tan=" <<tan(theta_r)<<" \tdetla =  "<<delta;
      img_i_ = 0;
      img_j_ = img_center_j_ - delta;
    }
    else if( theta > d && theta < 360)
    {
      //std::cout<<"\n Theta more than 315";
      //std::cout<<"\n Case A-Db";
      delta = (img_h_/2) * tan(theta_r);

      img_i_ = 0;
      img_j_ = img_center_j_ - delta;
    }
    else if (theta >a and theta<b)
    {
      //std::cout<<"\n Theta A B";
      //std::cout<<"\n Case A-B";
      float new_theta_r = (M_PI/2) - theta_r;
      delta = (img_w_/2) * tan(new_theta_r);

      img_i_ = img_center_i_ - delta;
      img_j_ = 0;
    }
    else if ( theta >b && theta <c){
      //std::cout<<"\n Theta B C";
      //std::cout<<"\n Case B-C";
      float new_theta_r = M_PI - theta_r ;
      delta = (img_h_/2) * tan(new_theta_r);

      img_i_ = img_h_;
      img_j_ = img_center_j_ - delta;
    }
    else if ( theta >c && theta <d){
      //std::cout<<"\n Theta C D";
      //std::cout<<"\n Case D-C";
      float new_theta_r = (3*M_PI/2)-theta_r;
      delta = (img_w_/2) * tan(new_theta_r);

      img_i_ = img_center_i_ + delta;
      img_j_ = img_w_;
    }
    else {
      if(theta == a){
        //std::cout<<"\n Case A";
        img_i_ = 0 ;
        img_j_ = 0 ;
        //std::cout<<"\n A";
      }
      else if (theta ==b){
        //std::cout<<"\n Case B";
        img_i_ = img_h_ ;
        img_j_ = 0 ;
        //std::cout<<"\n B";
      }
      else if(theta ==c){
        //std::cout<<"\n Case C";
        img_i_ = img_h_ ;
        img_j_ = img_w_ ;
        //std::cout<<"\n C";
      }
      else if(theta ==d){
        //std::cout<<"\n Case D";
        img_i_ = 0 ;
        img_j_ = img_w_ ;
        //std::cout<<"\n D";
      }
      else{std::cout<<"\n A B C D";}

    }
    std::vector<int> location; // storing i,j of grid boundary along each angle direction
    location.emplace_back(img_i_);
    location.emplace_back(img_j_);

    double d = sqrt(pow((img_i_ - img_center_i_),2) + pow((img_j_ - img_center_j_),2)) / img_resolution_;
    Hash_loc.insert(std::make_pair(theta,location));
    Hash_angl.insert(std::make_pair(theta,d));
  }
    std::cout<<"\n HashMap initialized";
}

int main(int argc, char** argv){
  return 0;
}
