#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <stack>  
// ------------------------------------------------------------------///
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


using namespace std;

//==================== for controller ============================//

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

    

    void call_shutdown()
    {
      ROS_INFO("Shutdown time");
    }


    void counterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("Getting current position");
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
     ROS_INFO("Current Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Current Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Current Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    
    prev_x=current_x;
    prev_y=current_y;
    current_x=pose2d.x;
    current_y=pose2d.y;
    q_z=msg->pose.pose.orientation.z;
    q_w=msg->pose.pose.orientation.w;

    }


    void Move(float xdes,float ydes)
    {
      ROS_INFO("Entered Move");
      cout<<"Received co-ordinates  x : "<<xdes<<" y : "<<ydes<<endl;
      ros::NodeHandle n;
      //ros::Rate rate(5);
      
      ros::Publisher forward_pub=n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",10);
      
      ros::Subscriber sub=n.subscribe("/odom", 10, counterCallback);
      
      ROS_INFO("Current Position-> x: [%f], y: [%f]", current_x,current_y);
      //double x_des = 5; //desired coordinates
      //double y_des = 5;
      double x_des=xdes;
      double y_des=ydes;
      double tolerance = 0.1; //value to assess destination is reached
      double kp_lin = 0.001; //linear positional
      double kp_ang = 0.6; //angular positional 
      double kd_lin = 100; //linear differential
      double kd_ang = 0.1; //angular differential
      double ki_lin = 0.00001; //linear integral
      double ki_ang = 0.001; //angular integral
      float freq = 100; //rate at which simulation is run
      
      geometry_msgs::Twist new_pub;

      double prev_linear_error;
      double prev_angular_error=std::atan2((y_des-current_y),(x_des-current_x))-std::atan2((current_y-prev_y),(current_x-prev_x));
      double sum_linear_error=0,sum_angular_error=0;
      double temp_d_linear=0;
      double current_linear_error,current_angular_error,d_linear_error,d_angular_error;

      while(ros::ok())
      {
        ROS_INFO("Started to move");

        //ros::spinOnce(); 
          new_pub.linear.x=0.3;
          current_linear_error=std::sqrt(std::pow((x_des-current_x),2)+std::pow((y_des-current_y),2));
          ROS_INFO("current_error = [%lf]",current_linear_error);
          ROS_INFO("prev_linear_error = [%lf]",prev_linear_error);
          d_linear_error=(current_linear_error-prev_linear_error)*freq;
          
          if(d_linear_error==0)
          {
              d_linear_error=temp_d_linear;
          }
          prev_linear_error=current_linear_error;

          sum_linear_error=sum_linear_error+current_linear_error;
          double linear_velocity=kp_lin*current_linear_error+ki_lin*sum_linear_error+kd_lin*d_linear_error;

          double theta1 = std::atan2(2*(q_w*q_z) , (1 - 2*q_z*q_z));

          current_angular_error=std::atan2((y_des-current_y),(x_des-current_x))-theta1;

          d_angular_error=(current_angular_error-prev_angular_error)*freq;

          prev_angular_error=current_angular_error;

          sum_angular_error=sum_angular_error+current_angular_error;

          double angular_velocity=kp_ang*current_angular_error+kd_ang*d_angular_error+ki_ang*sum_angular_error;

          //geometry_msgs::Twist new_pub;

          new_pub.angular.z=angular_velocity;


          forward_pub.publish(new_pub);
          
          ROS_INFO("Current Position-> x: [%f], y: [%f]", current_x,current_y);
          
          ROS_INFO("Desired Position-> x_des: [%f], y_des: [%f]", x_des,y_des);
          ROS_INFO(" d_linear_error = [%lf]",d_linear_error);
          ROS_INFO("sum_linear_error = [%lf]",sum_linear_error);
          ROS_INFO("PID linear vlaue -- [%lf]",linear_velocity);
          ROS_INFO("PID angular vlaue -- [%lf]",angular_velocity);
          ROS_INFO("temp term = [%lf] , [%lf]", x_des-current_x,y_des-current_y);
          if(linear_velocity < 0)
          { 
            ROS_INFO("PID greater than 0");
            new_pub.linear.x=std::max(linear_velocity,0.3);
          }
          else
          {
            new_pub.linear.x=std::min(linear_velocity,0.00);
            if(current_x > x_des && current_y > y_des)
            {
              new_pub.angular.z=0.00;
            }
            ROS_INFO("linear velocity = [%lf]",new_pub.linear.x);
            ROS_INFO("Reached waypoint");
            ROS_INFO("PID vlaue -- [%lf]",linear_velocity);
            ROS_INFO("PID less than 0");
            //call_shutdown();
            break;
          }

          forward_pub.publish(new_pub);
          
          // if(current_x < x_des)
          // {
          //   ROS_INFO("Not Reached");
          
          // }
          // else{
          //   ROS_INFO("Reached destination");
          //   //ROS_INFO("Current Position-> x: [%f], y: [%f]", current_x,current_y);
          //   new_pub.linear.x=0;
          //   forward_pub.publish(new_pub);
          //   ROS_INFO(" d_linear_error = [%lf]",d_linear_error);
          //   ROS_INFO("sum_linear_error = [%lf]",sum_linear_error);
          //   ROS_INFO("PID vlaue -- [%lf]",linear_velocity);
            
          //   call_shutdown();
          // }
          temp_d_linear=d_linear_error;
          ros::Rate loop_rate(100);
          loop_rate.sleep();
          ros::spinOnce(); 

      }
    
  }




//========================================================//


class Sensing{
   
  public:
    double grid_resolution;
    int grid_size;
    int grid_connections;

    struct sNode{
      bool bObstacle = false;       // Is the node an obstruction?
      bool bVisited = false;        // Have we searched this node before?
      float fGlobalGoal;            // Distance to goal so far
      float fLocalGoal;             // Distance to goal if we took the alternative route
      float x;                        // Nodes position in 2D space
      float y;

      vector<sNode*> vecNeighbours; // Connections to neighbours
      sNode* parent;                // Node connecting to this node that offers shortest parent
    };

    vector< vector<sNode> > nodes; // initializing map to represent all nodes
    stack < pair<float,float> > path; // store path top: current node, bottom: goal node

    // Controller variables
    pair<int, int> state; // Current State
    pair<int, int> prev_state; // Previous State

    Sensing(ros::NodeHandle &nh);  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap
    bool solve_astar();
    void printPath();
    void getPath();
    void printPathNodes(stack < pair<float,float> > path);
    // void PIDController();
    // void RobotState();

  private:
    void make_connections();      // add neighbors to the nodes 

};

Sensing::Sensing(ros::NodeHandle &nh){ //constructor

  ROS_INFO("Sensing node initialized ...");
  if(nh.hasParam("costmap_node/costmap/width")){
    nh.getParam("costmap_node/costmap/width", grid_size);
    nh.getParam("costmap_node/costmap/resolution", grid_resolution);
    nh.getParam("robot_info/grid_connections", grid_connections);
  }  
  else
    ROS_ERROR("Did not find parameters");

  grid_size /= grid_resolution;

  nodes.resize(grid_size, vector<sNode>(grid_size));   // allocating memory to initialized vector 
  
  // Add neighbors
  make_connections();
}

void Sensing::make_connections(){

  ROS_INFO(" Solving for neighbours of node ...");
  // Add neighbors

  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){
      
      // 4-connected grid
      if (i>0)
        nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j+0]);
      if (i<grid_size)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j+0]);
      if (j>0)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+0][j-1]);
      if (j<grid_size)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+0][j+1]);

      if(grid_connections == 8){
        // 8-connected grid
        if (i>0 && j>0)
          nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j-1]);
        if (i<grid_size && j>0)
          nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j-1]);
        if (i>0 && j<grid_size)
          nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j+1]);
        if (j<grid_size && j<grid_size)
          nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j+1]);
      }
    
    } // j
  } // i
}


bool Sensing::solve_astar(){
  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[20][20];

  cout << "Solve astar started" << endl;
  ROS_INFO("Solving Astar started ...");
  // Reset Navigation Graph - default all node states
  for (int x = 0; x < grid_size; x++)
    for (int y = 0; y < grid_size; y++)
    {
      nodes[x][y].bVisited = false;
      nodes[x][y].fGlobalGoal = INFINITY;
      nodes[x][y].fLocalGoal = INFINITY;
      nodes[x][y].parent = nullptr;  // No parents
    }

    auto distance = [](sNode* a, sNode* b){ // For convenience
      return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    };

    auto heuristic = [distance](sNode* a, sNode* b){ // So we can experiment with heuristic
      return distance(a, b);
    };

    // Setup starting conditions
    sNode *nodeCurrent = nodeStart;
    nodeStart->fLocalGoal = 0.0f;
    nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

    // Add start node to not tested list - this will ensure it gets tested.
    // As the algorithm progresses, newly discovered nodes get added to this
    // list, and will themselves be tested later
    list<sNode*> listNotTestedNodes;
    listNotTestedNodes.push_back(nodeStart);

    // if the not tested list contains nodes, there may be better paths
    // which have not yet been explored. However, we will also stop 
    // searching when we reach the target - there may well be better
    // paths but this one will do - it wont be the longest.
    while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
    {
      // Sort Untested nodes by global goal, so lowest is first
      listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );
      
      // Front of listNotTestedNodes is potentially the lowest distance node. Our
      // list may also contain nodes that have been visited, so ditch these...
      while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
        listNotTestedNodes.pop_front();

      // ...or abort because there are no valid nodes left to test
      if (listNotTestedNodes.empty())
        break;

      nodeCurrent = listNotTestedNodes.front();
      nodeCurrent->bVisited = true; // We only explore a node once
      
          
      // Check each of this node's neighbours...
      for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
      {
        // ... and only if the neighbour is not visited and is 
        // not an obstacle, add it to NotTested List
        if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == false)
          listNotTestedNodes.push_back(nodeNeighbour);

        // Calculate the neighbours potential lowest parent distance
        float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

        // If choosing to path through this node is a lower distance than what 
        // the neighbour currently has set, update the neighbour to use this node
        // as the path source, and set its distance scores as necessary
        if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
        {
          nodeNeighbour->parent = nodeCurrent;
          nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

          // The best path length to the neighbour being tested has changed, so
          // update the neighbour's score. The heuristic is used to globally bias
          // the path algorithm, so it knows if its getting better or worse. At some
          // point the algo will realise this path is worse and abandon it, and then go
          // and search along the next best path.
          nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
        }
      } 
    }

    return true;
}


void Sensing::printPath(){
  float x,y;

  //Move();
  stack<pair<float,float>> pathreq;
  ROS_INFO("Print path started ...");
  
  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[20][20];
  sNode *p = nodeEnd;
  
  while (p->parent != nullptr)
  {
    x=p->x;
    y=p->y;
    //cout << "x: " << p->x << " y: " << p->y<<"\n"<<endl; 
    pathreq.push((make_pair(p->x,p->y)));
    // Set next node to this node's parent
    p = p->parent;
  }

  printPathNodes(pathreq);
  cout <<"\n"<< endl;
  
}

void Sensing::getPath(){

  ROS_INFO("GetPath started...");
  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[20][20];
  sNode *p = nodeEnd;
  
  while (p->parent != nullptr)
  {
    path.push(make_pair(p->x,p->y)); 
    // Set next node to this node's parent
    p = p->parent;
  }

  printPathNodes(path);
}

void Sensing::printPathNodes(stack < pair<float,float> > path)
{
  ROS_INFO("Printing the path through printPathNodes...");
  std::pair<float,float> coor;
  while(!path.empty())
  {
    coor=path.top();
    std::cout<<"x : "<<coor.first<<" "<<"y : "<<coor.second<<std::endl;
    //Move(coor.first,coor.second);
    path.pop();
  }
  
  
}

// void Sensing::RobotStateCbk(nav_msgs::Odometry::ConstPtr msg){
  
//   state.first  = msg->pose.position.x;
//   state.second = msg->pose.position.y;

// }

// void Sensing::PIDController(){


// }

void Sensing::costmapCb(const nav_msgs::OccupancyGridConstPtr grid){ 

  ROS_INFO("costmapCb node started .....");
  
  grid_resolution = grid->info.resolution;   
 
  float map_x = grid->info.origin.position.x;
  float map_y = grid->info.origin.position.y; 

  auto map =  grid->data; 
  
  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){      
    
      nodes[i][j].x = map_x + i*grid_resolution; 
      nodes[i][j].y = map_y + j*grid_resolution;

      // Updating obstacle information
      if ((int) map[grid_size*j+i] == 100){ // 100 = obstacle
        nodes[i][j].bObstacle = true;
      }   
    }//j
  } //i 


  // cout << nodes[1][1].x << endl;
  // solve_astar();
  // printPath();

  if(solve_astar())
  {
    printPath();
    getPath();
  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  Sensing sensing(n);

  ros::Subscriber sub_costmap = n.subscribe("/costmap_node/costmap/costmap", 100000, &Sensing::costmapCb, &sensing);
  
  ros::spin();
  
  return 0;
}




  