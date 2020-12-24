
# include "hierarchical_motion_planner/trajectory_executor.h"


TrajectoryExecutor::TrajectoryExecutor(ros::NodeHandle& nh) : _nh(nh){

    _nh.getParam("/hierarchical_planner/planner", _planner);

    _controller = new PID(_nh);
}


TrajectoryExecutor::~TrajectoryExecutor(){
    delete _controller;
}

void TrajectoryExecutor::setPath(std::vector<geometry_msgs::Point>& path){
    _path = path;
}


std::vector<geometry_msgs::Point> TrajectoryExecutor::executePath(){

    bool pointReachedSuccessfully{true};

    while( _goalIndex != _path.size() && pointReachedSuccessfully){
        
        std::cout << " Size of trajectory Vector = " << _path.size() << "\n";
        std::cout << " Goal Index = " << _goalIndex << "\n";

        _controller->setGoal(_path[_goalIndex]);

        pointReachedSuccessfully = _controller->execute();

        // to check if the node works on detecting an obstacle
        // if(_goalIndex == 1){
        //     pointReachedSuccessfully = false;
        // }


        if(pointReachedSuccessfully)
            _goalIndex++;
        else
            break;
    }

    if(!pointReachedSuccessfully){
        ROS_INFO_STREAM("The Robot found an obstacle along the " + _planner + "path");
    }

    _updatePath();
    return _path;
}


std::vector<geometry_msgs::Point> TrajectoryExecutor::getPath(){
    return _path;
}


void TrajectoryExecutor::_updatePath(){
    _path.erase(_path.begin(), _path.begin() + _goalIndex);   
}


void TrajectoryExecutor::printPathVector(std::vector<geometry_msgs::Point>& path){
    size_t pathSize = path.size();

    if(!pathSize){
        std::cout << "Path is empty \n";
        return;
    }

    std::cout << "Left to execute the following goal points \n";
    for(int i=0; i<pathSize; i++){
        std::cout << "Goal" << i+1 << " (" << path[i].x << " , " << path[i].y << ")\n";
    }
}


int main(int argc, char** argv){

  ros::init(argc, argv, "trajectory_executor_node");

  ros::NodeHandle nh;

  nh.setParam("/hierarchical_planner/planner", "global");
  nh.setParam("/hierarchical_planner/is_robot_colliding", false);
  std::vector<geometry_msgs::Point> path;

  geometry_msgs::Point goal, goal2, goal3, goal4;
  goal.x = 6.0;
  goal.y = 3.0;
  path.push_back(goal);

  goal2.x = 3.0;
  goal2.y = 6.0;
  path.push_back(goal2);
  

  goal3.x = -5.0;
  goal3.y = 5.0;
  path.push_back(goal3);

  goal4.x = 0.0;
  goal4.y = 0.0;
  path.push_back(goal4);


  TrajectoryExecutor trajEx(nh);
  trajEx.setPath(path);
  auto finalPath = trajEx.executePath();

  if(!finalPath.size())
    ROS_INFO("The Robot completed executing the set-point trajectory");

  else
    ROS_WARN("Obstacle encountered");

  trajEx.printPathVector(finalPath);
  
  return 0;
}

