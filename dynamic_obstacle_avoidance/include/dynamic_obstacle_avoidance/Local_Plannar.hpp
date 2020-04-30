#pragma once

//#include<iostream>
//#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<math.h>
#include"dynamic_obstacle_avoidance/Map_Manager.hpp"
//#include<vector>


class local_planner{
public:
  local_planner(Map_Manager manager, int step_size);
  std::vector<geometry_msgs::PoseStamped> makePlan(std::vector<float> root_node, std::vector<float> target_node);

private:
  struct node{
    std::vector<float> world_c_ ; // vector of <x,y> points in world (robot) coordinate frame
    std::vector<int> img_c_ ; // vector of <i,j> points in image coordinate frame
    double costToCome_;
    std::vector<int> children_indices_;  // vector of int <indices> to store the index of all the children of a given node
  };

  std::vector<node> tree;
  std::vector<std::vector<int>> Cfree_;
  float region_radius_;
  int branch_length;
  float distance_to_goal_;
  Map_Manager regional_manager_;
  vector<geometry_msgs::PoseStamped> path_reversed_;
  vector<geometry_msgs::PoseStamped> path_;

  float calculateDistance_(std::vector<int> first_point, std::vector<int> second_point); // KAVIT
  std::vector<int> get_random_point_(); // generates random vector <i,j> in image coordinate frame //SOUMYA
  bool hasObstacle_(std::vector<int> Xnear, std::vector<int> Xnew); // checks if the straight_line path between Xnear(<i,j>) and Xnew (<i,j>) is free from obstacles (true = hasObstacle & false=ObstacleFree)  //KAVIT
  std::vector<int> newNode_(std::vector<int>Xnear , std::vector<int> Xrand);  // creates a new node (vector in <i,j>) in between Xnear(node of tree <i,j>) ,Xrand(randomly sampled point <i,j> // KAVIT
  std::vector<int> getNeighbourhood_(std::vector<int> Xnew);  // returns a vector of integers <indices__of_nodes_in_tree> which are inside the region_radius around the point Xnew<i,j>; // SOUMTA
  long getBestParent_(std::vector<int> neighbourhood); // it returns the index of best parent for a given node X_new whose neighbourhood (indices of nearby nodes) is given to it as input args  //SOUMYA
  long findParent_(long child_index); //this node returns the index of parent node for the given child node passed to it in args. The parent is seached from looking thru 'children_indices_' in the struct of tree nodes  //SOUMYA
  void reversePath_(); // store the plan from path_reverse into path in reverse order  //KAVIT
};
