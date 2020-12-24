
#include"hierarchical_motion_planner/global_planner2.h"

using namespace std;


GlobalPlanner::GlobalPlanner()
{
    ROS_INFO("Default constructor");
}

GlobalPlanner::GlobalPlanner(ros::NodeHandle &nh /*,std::vector<float> start_point , std::vector<float> end_point*/){
    pub_path_ = nh.advertise<hierarchical_motion_planner::Trajectory>("global_path",1);

    ROS_INFO("Sensing node initialised...");
    if(nh.hasParam("costmap_node/costmap/width")){
        nh.getParam("costmap_node/costmap/width",grid_size_);
        nh.getParam("costmap_node/costmap/resolution",grid_resolution_);
        nh.getParam("planner/grid_connections",grid_connections_);
        //nh.getParam("planner/goal",goal);
        //nh.getParam("planner/start",start);
        //ROS_INFO("Got parameters");
        //std::cout<<goal[0]<<goal[1]<<std::endl;

    }
    else{
        ROS_ERROR("Did not find parameters");
    }
    
    grid_size_ /= grid_resolution_;

    nodes_.resize(grid_size_, std::vector<sNode>(grid_size_));
    std::cout << "Nodes size : " << nodes_.size() << "\n";
    make_connections_();

}

/* Pass in the start and goal array to the A* planner class through this function */
void GlobalPlanner::setTargets(float start_pt[2] , float goal_pt[2]){
    ROS_INFO("Set Targets");

    current_ = std::vector<float>{start_pt[0], start_pt[1]};
    goal_ = std::vector<float>{goal_pt[0], goal_pt[1]};

}

void GlobalPlanner::get_start_end_nodes(){
    ROS_INFO("Get Start End Nodes");

    std::cout << current_[0] << "\n";

    std::cout << "Map x,y = " << map_x_ << " " << map_y_ << "\n";
    std::cout << "Grid size = " << grid_size_ << "\n";
    std::cout << "Grid Resol = " << grid_resolution_ << "\n";
    std::cout << "Grid Connec = " << grid_connections_ << "\n";

    int start_node_i = floor((current_[0] - map_x_)/ grid_resolution_);
    int start_node_j = floor((current_[0] - map_y_)/ grid_resolution_);

    int goal_node_i = floor((goal_[0]-map_x_)/grid_resolution_);
    int goal_node_j = floor((goal_[1]-map_y_)/grid_resolution_);

    nodeStart_ = &nodes_[start_node_i][start_node_j];
    nodeEnd_ = &nodes_[goal_node_i][goal_node_j];
    
    std::cout<<"start=> x: " << current_[0] << " y: " << current_[1] << std::endl;
    std::cout<<"start=> i: " << start_node_i << " j: " << start_node_j << std::endl;
    std::cout<<"goal => x: " << goal_[0] << " y: " << goal_[1] << std::endl;
    std::cout<<"goal => i: " << goal_node_i << " j: " << goal_node_j << std::endl;

}

void GlobalPlanner::make_connections_(){

    ROS_INFO("Make Connections");

    ROS_INFO("finding neighbours of nodes");

    for(int i=0;i<grid_size_;i++){
        for(int j=0; j<grid_size_; j++){

        // 4-connected grid
        if (i>0)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i-1][j+0]);
        if (i<grid_size_)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i+1][j+0]);
        if (j>0)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i+0][j-1]);
        if (j<grid_size_)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i+0][j+1]);

        if(grid_connections_ == 8){
            // 8-connected grid
            if (i>0 && j>0)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i-1][j-1]);
            if (i<grid_size_ && j>0)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i+1][j-1]);
            if (i>0 && j<grid_size_)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i-1][j+1]);
            if (j<grid_size_ && j<grid_size_)
            nodes_[i][j].vecNeighbours.push_back(&nodes_[i+1][j+1]);
            }
        }
    }

    ROS_INFO("Found neighbours");
}

bool GlobalPlanner::solve_astar(){

    ROS_INFO("Solve astar started");

    get_start_end_nodes();
    std::cout<<"start=> x: " << nodeStart_->x << " y: " << nodeStart_->y << std::endl;
    std::cout<<"end=> x: " << nodeEnd_->x << " y: " << nodeEnd_->y << std::endl;



    // Set default value for all nodes
    for(int x=0;x < grid_size_;x++){
        for(int y=0; y<grid_size_;y++){
            nodes_[x][y].bVisited = false;
            nodes_[x][y].fGlobalGoal = INFINITY;
            nodes_[x][y].fLocalGoal = INFINITY;
            nodes_[x][y].parent = nullptr; //no parents

        }

    }

    auto distance = [](sNode* a, sNode* b){
        return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    };

    auto heuristic = [distance](sNode* a, sNode* b){
        return distance(a,b);
    };
    //std::cout<<"setting up starting conditions"<<std::endl;
    //Setup starting conditions
    sNode *nodeCurrent = nodeStart_;
    nodeStart_->fLocalGoal = 0.0f;
    nodeStart_->fGlobalGoal = heuristic(nodeStart_,nodeEnd_);

    // Add start node to not tested list - this will ensure it gets tested.
    // As the algorithm progresses, newly discovered nodes get added to this
    // list, and will themselves be tested later

    std::list<sNode*> listNotTestedNodes;
    listNotTestedNodes.push_back(nodeStart_);
    //ROS_INFO("starting node pushed");
    // if the not tested list contains nodes, there may be better paths
    // which have not yet been explored. However, we will also stop
    // searching when we reach the target - there may well be better
    // paths but this one will do - it wont be the longest.

    //std::cout<<"Finding out neighbours and starting"<<std::endl;
    while(!listNotTestedNodes.empty() && nodeCurrent!=nodeEnd_)
    {
         // Find absolutely shortest path // && nodeCurrent != nodeEnd)
        // Sort Untested nodes by global goal, so lowest is first
        listNotTestedNodes.sort([](const sNode* lhs,const sNode* rhs)
        {return lhs->fGlobalGoal < rhs->fGlobalGoal;}
        );


        // Front of listNotTestedNodes is potentially the lowest distance node. Our
      // list may also contain nodes that have been visited, so ditch these...
        while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
            listNotTestedNodes.pop_front();

        // ...or abort because there are no valid nodes left to test
        if(listNotTestedNodes.empty())
         break;

        nodeCurrent = listNotTestedNodes.front();
        nodeCurrent->bVisited = true; // we only express a node once

        //Check for the node's neighbours...
        for(auto nodeNeighbour : nodeCurrent->vecNeighbours)
        {
            // ... and only if the neighbour is not visited and is
            // not an obstacle, add it to NotTested List

            if(!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
                listNotTestedNodes.push_back(nodeNeighbour);

            // Calculate the neighbours potential lowest parent distance
            float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent,nodeNeighbour);

            // If choosing to path through this node is a lower distance than what
            // the neighbour currently has set, update the neighbour to use this node
            // as the path source, and set its distance scores as necessary

            if(fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
            {
                nodeNeighbour->parent = nodeCurrent;
                nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
            }

           // The best path length to the neighbour being tested has changed, so
          // update the neighbour's score. The heuristic is used to globally bias
          // the path algorithm, so it knows if its getting better or worse. At some
          // point the algo will realise this path is worse and abandon it, and then go
          // and search along the next best path.

          nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour,nodeEnd_);


        }


    }

    if(nodeCurrent == nodeEnd_){
        ROS_INFO("Found path to Goal");
        return true;
    }
    else if(listNotTestedNodes.empty()){
        ROS_ERROR("List got empty");
        return false;
    }

    //std::cout<< nodeEnd->parent<<std::endl;
    return true;



}

void GlobalPlanner::printPath(){
    ROS_INFO("printPath");

    std::cout<< "-----------------------------"<< std::endl;

    for(auto state : path_msg_.traj)
    {
        std::cout<<" x: "<<state.x<<" y: "<<state.y<<std::endl;;
        // Set next node to this node's parent
        // p = p->parent;
    }

    std::cout<< "-----------------------------"<<std::endl;
}

void GlobalPlanner::getPath(){
    ROS_INFO("Get Path");

    sNode *p = nodeEnd_->parent;

    hierarchical_motion_planner::State state_msg;
    path_msg_.traj.clear();

    state_msg.x = goal_[0];
    state_msg.y = goal_[1];
    path_msg_.traj.insert(path_msg_.traj.begin(), state_msg);
   // std::cout<<p->parent<<std::endl;

   while (p->parent != nullptr){
       state_msg.x = p->x;
       state_msg.y = p->y;

       path_msg_.traj.insert(path_msg_.traj.begin(),state_msg);

       //Set next node to this node's parent

       p=p->parent;
   }

   //planning_done = true;
}



void GlobalPlanner::publishPath(){
    ROS_INFO("Publishing path");
    if(!(path_msg_.traj).empty())
    {
        printPath();
        pub_path_.publish(path_msg_);
    }
    else
        ROS_ERROR("Path not Found  !!! ");
}


void GlobalPlanner::convertToStack()
{
    int length_path=(path_msg_.traj).size();
      for(int i=length_path-1;i>=0;i--)
      {
          path_found_.push((path_msg_.traj)[i]);
          //std::cout<<" x: "<<state.x<<" y: "<<state.y<<std::endl;
          //length=length+1;
          // Set next node to this node's parent
          // p = p->parent;
      }
    printStackPath(path_found_);


}

void GlobalPlanner::printStackPath(std::stack<hierarchical_motion_planner::State> path_stk)
{
    ROS_INFO("THe size of path is");
    std::cout<<(path_stk).size()<<std::endl;
    ROS_INFO("The stack of path is");
      while(!path_stk.empty())
      {
        hierarchical_motion_planner::State state_temp;
        state_temp=path_stk.top();
        std::cout<<"state_temp.x : "<<state_temp.x<<" state_temp.y : "<<state_temp.y<<std::endl;
        path_stk.pop();
      }
      b_planning_done_ = true;
}


stack<hierarchical_motion_planner::State> GlobalPlanner::start_astar()
{
    ROS_INFO("Starting A star");
    ros::NodeHandle n;

    //Sensing sensing(n,current, goal);

    ros::Subscriber sub_costmap = n.subscribe("/costmap_node/costmap/costmap",1,&GlobalPlanner::costmapCb_,this);
    ros::Subscriber subs_odom = n.subscribe("/odom",1, &GlobalPlanner::odomCb_,this);
    ros::Subscriber S = n.subscribe("/gazebo/model_states",1, &GlobalPlanner::gazeboModelStateCB, this);

    int PLANNING_FREQ = 1;

    n.getParam("planner/planning_freq", PLANNING_FREQ);

    ros::Rate loop_rate(100000000);
    int i=0;

    if(b_is_occ_map_initialized){
        ROS_INFO("Occ Grid Topic recvd");
        while(ros::ok() and (!b_planning_done_))
        {
            i++;
            b_planning_done_ = false;
            ros::spinOnce();
            //sensing.publishPath();
            ROS_INFO(" i = [%d]",i);
            ROS_INFO ("Here will run only after occ grid is recv ");
            loop_rate.sleep();
            if(i > 5)
            {
                break;
            }

            std::cout<<" length of path found in astar : "<<(this->path_msg_.traj).size() << std::endl;
            //ros::Duration(1000).sleep();
            // }

        }
    }
    else{
        while(!b_is_occ_map_initialized){
            ROS_INFO("waiting for occ grid to be active");
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }


    std::cout<<"Length of path in Astar : "<<(this->path_found_).size()<<std::endl;
    ROS_INFO("Finished A Star Function");
    return this->path_found_;

}

/*ROS Call Back Functions*/

void GlobalPlanner::odomCb_(nav_msgs::Odometry::ConstPtr msg)
{
    //ROS_INFO("Getting the current position");
    current_[0] = msg->pose.pose.position.x;
    current_[1] = msg->pose.pose.position.y;
    std::cout <<" current x : "<<current_[0]<<"current y : "<<current_[1]<<std::endl;
}


void GlobalPlanner::costmapCb_(const nav_msgs::OccupancyGridConstPtr grid)
{
    ROS_INFO("Costmap callback");
    grid_resolution_ = grid->info.resolution;

    //local map location in odom frame (fixed)

    map_x_ = grid->info.origin.position.x;
    map_y_ = grid->info.origin.position.y;
    std::cout << "Setting up costmaps from call back\n";
    std::cout << "map_x_ = " << map_x_ << "\n";
    std::cout << "map_y_ = " << map_y_ << "\n";
    std::cout << "grid_resol = " << grid_resolution_ << "\n";
    auto map = grid->data;

    //Define global nodes positions in terms of x and y

    for(int i {0}; i<grid_size_;i++){
        for(int j {0}; j<grid_size_;j++){

            nodes_[i][j].x = map_x_ + i*grid_resolution_ + grid_resolution_/2;
            nodes_[i][j].y = map_y_ + j*grid_resolution_ + grid_resolution_/2;

            //Updating obstacle information
            if((int) map[grid_size_*j+i] > 5){ // obstacle =100
                nodes_[i][j].bObstacle = true;
                //std::cout<<"X : "<<nodes_[i][j].x<<" Y : "<<nodes_[i][j].y<<std::endl;
            }
            else
                nodes_[i][j].bObstacle = false;
        }
    }

    /*bool b_GotPath = solve_astar();
    if(b_GotPath){
        ROS_INFO(" Got Path !!");
        getPath();
        convertToStack();
        //printPath();
    }
    else
        ROS_ERROR("A Start failed !! ");*/

    b_is_occ_map_initialized = true;
}



void GlobalPlanner::gazeboModelStateCB(const gazebo_msgs::ModelStates::ConstPtr& ptr){
    int robot_index = (ptr->name).size()-1;
    double robo_x = ((ptr->pose)[robot_index]).position.x;
    double robo_y = (ptr->pose)[robot_index].position.y;

    if(b_is_occ_map_initialized){

        int curr_node_i = floor((robo_x - map_x_)/ grid_resolution_);
        int curr_node_j = floor((robo_y - map_y_)/ grid_resolution_);
        std::cout << "=======================\n";
        std::cout << "From gazeboModelStateCB\n";
        std::cout << "robot_x = " << robo_x << "\n";
        std::cout << "robot_y = " << robo_y << "\n";
        std::cout << "map_x_ = " << map_x_ << "\n";
        std::cout << "map_y_ = " << map_y_ << "\n";
        std::cout << "grid_resol = " << grid_resolution_ << "\n";
        std::cout << "Robot Current Node => i: " << curr_node_i << " j: " <<  curr_node_j << std::endl;
        std::cout << "=======================\n";
    }
    else{
        std::cout << " Occ Grid Not yet up\n";
    }
}


 int main(int argc, char **argv){

     ros::init(argc,argv, "listener");
     ros::NodeHandle nh;

     GlobalPlanner globalplanner(nh);
     //ros::Subscriber sub_costmap = nh.subscribe("/costmap_node/costmap/costmap",1,&Sensing::costmapCb,&sensing);
     //ros::Subscriber subs_odom = nh.subscribe("/odom",1, &Sensing::odomCb,&sensing);

     int PLANNING_FREQ = 1;

     float start[2] {0.0f, 0.0f};
     float end[2] {3.0f, 3.0f}; 
     globalplanner.setTargets(start, end);
     globalplanner.get_start_end_nodes();
     
    
     auto stK = globalplanner.start_astar();
     

     
     /*nh.getParam("planner/planning_freq", PLANNING_FREQ);

     ros::Rate loop_rate(PLANNING_FREQ);
     int i=0;
     while(ros::ok())
     {
         i++;
         ros::spinOnce();
         globalplanner.publishPath();

         loop_rate.sleep();
         // if(i > 1){
         // ros::Duration(1000).sleep();
         // }
     }*/

     return 0;
 }