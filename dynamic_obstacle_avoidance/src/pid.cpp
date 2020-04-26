#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>


double current_x,prev_x=-1;
double current_y,prev_y=-1;
double q_z;
double q_w;
double current_theta;
bool received_InitPosition=false;
double POS_TOLERANCE = 0.1;
double ANGLE_TOLERANCE = 0.05;
double flPOS_REQUEST_RATE = 30.0;
geometry_msgs::Pose2D pose2d;

void counterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  ROS_INFO("Hello");
  // ROS_INFO("%f", msg->twist.twist.linear.x);
  
  pose2d.x = msg->pose.pose.position.x;
  pose2d.y = msg->pose.pose.position.y;
    
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
    
  pose2d.theta = yaw;
  //pub_pose_.publish(pose2d);
  // ROS_INFO("Current Seq: [%d]", msg->header.seq);
  // ROS_INFO("Current Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Current Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Current Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

  
  prev_x=current_x;
  prev_y=current_y;
  current_x=pose2d.x;
  current_y=pose2d.y;
  q_z=msg->pose.pose.orientation.z;
  q_w=msg->pose.pose.orientation.w;

}

void call_shutdown()
{
  ROS_INFO("Shutdown time");
}


void Move()
{
  ros::NodeHandle n;
  ros::Rate rate(5);
  
  ros::Publisher forward_pub=n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",10);
  ros::Subscriber sub=n.subscribe("/odom", 1000, counterCallback);
  //ROS_INFO("Current Position-> x: [%f], y: [%f]", current_x,current_y);
  float freq = 100; //rate at which simulation is run
  //rate = rospy.Rate(freq) 
  double x_des = 2; //desired coordinates
  double y_des = 0;
  double tolerance = 0.1; //value to assess destination is reached
  double kp_lin = 0.00001; //linear positional
  double kp_ang = 0.6; //angular positional 
  double kd_lin = 0.001; //linear differential
  double kd_ang = 0.1; //angular differential
  double ki_lin = 0.0001; //linear integral
  double ki_ang = 0.001; //angular integral

  //calculating the previous errors
  double prev_linear_error=std::sqrt(std::pow((x_des-current_x),2)+std::pow((y_des-current_y),2));
  double prev_angular_error=std::atan2((y_des-current_y),(x_des-current_x))-std::atan2((current_y-prev_y),(current_x-prev_x));
  double sum_linear_error=0,sum_angular_error=0;

  geometry_msgs::Twist pub_Twist;
  //forward_pub.publish(pub_Twist);

  double current_linear_error,current_angular_error,d_linear_error,d_angular_error;

  while(ros::ok())
  {
    //ROS_INFO("Current Position-> x: [%f], y: [%f]", current_x,current_y);
    current_linear_error=std::sqrt(std::pow((x_des-current_x),2)+std::pow((y_des-current_y),2));
    //ROS_INFO("Current linear error --> %lf", current_linear_error);
    d_linear_error=(current_linear_error-prev_linear_error)*freq;
    prev_linear_error=current_linear_error;

    sum_linear_error=sum_linear_error+current_linear_error;
    double linear_velocity=kp_lin*current_linear_error+ki_lin*sum_linear_error+kd_lin*d_linear_error;

    // double theta1 = std::atan2(2*(q_w*q_z) , (1 - 2*q_z*q_z));

    // current_angular_error=std::atan2((y_des-current_y),(x_des-current_x))-theta1;

    // d_angular_error=(current_angular_error-prev_angular_error)*freq;

    // prev_angular_error=current_angular_error;

    // sum_angular_error=sum_angular_error+current_angular_error;

    // double angular_velocity=kp_ang*current_angular_error+kd_ang*d_angular_error+ki_ang*sum_angular_error;

    //geometry_msgs::Twist new_pub;

    //pub_Twist.angular.z=angular_velocity;

    if(linear_velocity > 0)
    {
      pub_Twist.linear.x=std::min(linear_velocity,0.3);
    }
    else
    {
      pub_Twist.linear.x=std::min(linear_velocity,-0.3);
    }
    ROS_INFO("current position :  x-->  %lf,  y--> %lf",current_x,current_y);

    //ROS_INFO("%lf",current_angular_error);

    forward_pub.publish(pub_Twist);

    ROS_INFO("Current linear velocity---> %lf", linear_velocity);

    if(current_linear_error<POS_TOLERANCE)
    {
      ROS_INFO("Reached destination");
      call_shutdown();
    }
    ros::spinOnce();

  }

}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"pid_controller");
  ros::NodeHandle nh;
  // Run program until manually stopped
  
  ros::Rate loop_rate(5);
  Move();
  
  ros::spinOnce();

  loop_rate.sleep();

  return 0;
}

//void getInitialPosition