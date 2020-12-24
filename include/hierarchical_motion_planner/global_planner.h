//om
#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <list>
#include <stack>  

// ------------------------------------------------------------------///
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "hierarchical_motion_planner/State.h"
#include "hierarchical_motion_planner/Trajectory.h"


#include <gazebo_msgs/ModelStates.h>


/*class GlobalPlanner{
    public : 
        

        GlobalPlanner(ros::NodeHandle &nh);  // constructor
        void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap
        bool solve_astar();
        void printPath();
        void getPath();
        std::stack<hierarchical_motion_planner::State> start_astar();
        void printPathNodes(std::stack < std::pair<float,float> > path);
        // void PIDController();
        // void RobotState();

        struct sNode{
            bool bObstacle = false;       // Is the node an obstruction?
            bool bVisited = false;        // Have we searched this node before?
            float fGlobalGoal;            // Distance to goal so far
            float fLocalGoal;             // Distance to goal if we took the alternative route
            float x;                        // Nodes position in 2D space
            float y;

            std::vector<sNode*> vecNeighbours; // Connections to neighbours
            sNode* parent;                // Node connecting to this node that offers shortest parent
        };

  
    private:
        double grid_resolution_;
        int grid_size_;
        int grid_connections_;

        bool b_planning_done_ = {false};

        std::vector< std::vector<sNode> > nodes_; // initializing map to represent all nodes
        std::stack < std::pair<float,float> > path_; // store path top: current node, bottom: goal node
        std::vector< std::pair<float, float>> Vec_path_;

        // Controller variables
        std::pair<int, int> state_; // Current State
        std::pair<int, int> prev_state_; // Previous State

        void make_connections_();      // add neighbors to the nodes 

        int node_min_i_{10}, node_min_j_{10};
        int node_max_i_{14}, node_max_j_{15};


};
*/

class GlobalPlanner{

    public:
        
        struct sNode{
            bool bObstacle = false;
            bool bVisited = false;
            float fGlobalGoal;
            float fLocalGoal;
            float x;
            float y;

            std::vector<sNode*> vecNeighbours;
            sNode* parent;
        };

        //GlobalPlanner(ros::NodeHandle &nh,std::vector<float>, std::vector<float>);
        GlobalPlanner(ros::NodeHandle &nh);
        GlobalPlanner();
        
        void setTargets(float[2], float[2]);
        std::stack<hierarchical_motion_planner::State> start_astar();
        bool solve_astar();
        void printPath();
        void getPath();
        void publishPath();
        void get_start_end_nodes();
        

    private:
        double grid_resolution_;

        int grid_size_;
        int grid_connections_;

        std::vector<float> current_;
        std::vector<float> goal_;

        float map_x_ {0};
        float map_y_ {0};

        std::vector< std::vector<sNode> > nodes_;


        sNode *nodeStart_ {};
        sNode *nodeEnd_ {};

        std::vector< std::pair<float,float> > path_;

        ros::Publisher pub_path_;
        hierarchical_motion_planner::Trajectory path_msg_;
        std::stack<hierarchical_motion_planner::State> path_found_;

        void costmapCb_(const nav_msgs::OccupancyGridConstPtr grid);
        void odomCb_(nav_msgs::Odometry::ConstPtr msg);

        void init_global_map_();

        void make_connections_();

        bool b_planning_done_ = false;
        //void printStackPath();
        void convertToStack();
        void printStackPath(std::stack<hierarchical_motion_planner::State> path_found);

        //temp
        void gazeboModelStateCB(const gazebo_msgs::ModelStates::ConstPtr& ptr);
        bool b_is_occ_map_initialized{false};

};
#endif