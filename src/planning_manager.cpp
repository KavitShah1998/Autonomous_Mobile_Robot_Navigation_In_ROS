// om

# include "hierarchical_motion_planner/planning_manager.h"


PlanningManager::PlanningManager(ros::NodeHandle& nh) : _nh(nh){

    _globalPlanner = std::make_unique<GlobalPlanner>(_nh);

    _trajectoryController = std::make_unique<TrajectoryExecutor>(_nh);
}


PlanningManager::~PlanningManager(){

}


void PlanningManager::setStartAndGoal(std::vector<double> start, 
                                      std::vector<double> goal){
    
    _globalPlanner->setGoal(goal);


}


void PlanningManager::execute(){

    _path.clear();
    _path = _globalPlanner->makePlan();

    std::cout << "PlanningManager::Execute()\n";
    for(int i=0; i<_path.size(); i++){
        std::cout << _path[i][0] << "," << _path[i][1] << "\n";
    }

    _trajectoryController->setPath(_getTrajectoryFromPath(_path));
    auto remainderPath = _trajectoryController->executePath();
}



// Convert the vector<vector<double>> path into a vector of geometry::msgs Point
std::vector<geometry_msgs::Point> PlanningManager::_getTrajectoryFromPath(const std::vector<std::vector<double>>& path){


    // create a vector and reserve size
    std::vector<geometry_msgs::Point> trajectory;
    trajectory.reserve(path.size());


    // create points and add it to vector
    for(int i=0; i<path.size(); i++){

        geometry_msgs::Point point;
        point.x = path[i][0];
        point.y = path[i][1];

        trajectory.push_back(point);
    }


    return trajectory;
}


void PlanningManager::_getPathFromTrajectory(){

}


// TODO 
// 1. Optimize Trajectory
// 2. Create obstacle checker node
// 3. Create a trajectory control PID / MPC

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_manager_node");
    
    ros::NodeHandle nh;

    PlanningManager pm(nh);

    std::vector<double> start{0.0, 0.0};
    std::vector<double> goal{0.0, -7.0};

    pm.setStartAndGoal(start, goal);
    pm.execute();
}