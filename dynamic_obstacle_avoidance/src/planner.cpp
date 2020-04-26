#include "planner.h"
#include "nav_msgs/OccupancyGrid.h"

Planner::Planner(){
	sub_costmap = n.subscribe("/costmap_node/costmap/costmap", 1, &Sensing::costmapCb, &sensor);
}

void Planner::Astar(){


}

void Planner::move(){
	// ROS_INFO("Robot moving . . .");
}




