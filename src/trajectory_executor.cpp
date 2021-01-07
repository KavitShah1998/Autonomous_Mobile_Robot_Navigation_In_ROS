
# include "hierarchical_motion_planner/trajectory_executor.h"


TrajectoryExecutor::TrajectoryExecutor(ros::NodeHandle& nh) : _nh(nh){

    _nh.getParam("/hierarchical_planner/planner", _planner);

    _controller = new PID(_nh);
}


TrajectoryExecutor::~TrajectoryExecutor(){
    delete _controller;
}

void TrajectoryExecutor::setPath(std::vector<geometry_msgs::Point> path){
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

