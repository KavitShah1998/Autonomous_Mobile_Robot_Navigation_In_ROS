//om

# include "hierarchical_motion_planner/local_planner.h"





LocalPlanner::LocalPlanner(){

    _loadParams();
    std::cout << "Started LP \n";
    // setup Occupancy grid subscriber
    _occupancyGridSubscriber = _nh.subscribe(
                                "/move_base/local_costmap/costmap", 1, 
                                &LocalPlanner::_occupancyGridCallback, this);

    
    while(!_b_isOccupancyGridImageInitialized){
        ros::spinOnce();
    }

    std::cout << "Reached A \n"; 
    _LocalPlannerAStar = std::make_unique<AStar>(_occupancyGridImage);
    _LocalPlannerAStar->setParams(_gridConnections, _obstacleThreshold);
    std::cout << "Reached B \n";
}





LocalPlanner::LocalPlanner(ros::NodeHandle& nh) : _nh(nh){
    std::cout << "Started LP \n";

    _loadParams();
    // setup Occupancy grid subscriber
    _occupancyGridSubscriber = _nh.subscribe(
                                "/move_base/local_costmap/costmap", 1, 
                                &LocalPlanner::_occupancyGridCallback, this);

    
    // Using costmap params decide which algorithm is LocalPlanner : {A*, RRT*}
    //{TODO}

    // using if-else logic create the instance of local planner
    //{TODO}

    while(!_b_isOccupancyGridImageInitialized){
        ros::spinOnce();
    }
    std::cout << "Reached A \n"; 
    _LocalPlannerAStar = std::make_unique<AStar>(_occupancyGridImage);
    _LocalPlannerAStar->setParams(_gridConnections, _obstacleThreshold);
    std::cout << "Reached B \n";
}





LocalPlanner::~LocalPlanner(){

}




/* Gets the current robot position from ROS */
void LocalPlanner::_getCurrentRobotPosition() {

    ros::Subscriber _odomSub = _nh.subscribe("/odom", 1, 
                                    &LocalPlanner::_odomCallback, this);
    
    // wait till callback is setup
    while(!_b_isOdomCallbackInitialized){
        ros::spinOnce();
    }
}




/* Odometry callback function: extracts robot's pose */
void LocalPlanner::_odomCallback(const nav_msgs::Odometry::ConstPtr& odometry){

    std::cout << " Received Odom callback\n";
    double x = odometry->pose.pose.position.x;
    double y = odometry->pose.pose.position.y;

    _startPos_XY.clear();
    _startPos_XY = std::vector<double>{x,y};
    
    _b_isOdomCallbackInitialized = true;
}




/* initializes start and goal positions */
void LocalPlanner::setGoal(std::vector<double> goal) {

    // set start point
    _getCurrentRobotPosition();

    // set goal point
    _goalPos_XY = goal;


    while(!_b_isOccupancyGridImageInitialized){
        ros::Duration(1).sleep();
    }

    // convert start and goal from m to pixels
    _initializeStartAndGoalInPixel();


    ROS_INFO_STREAM( _startPos_IJ[0] << " " << _startPos_IJ[1] << " Pix: " << (int)_occupancyGridImage.at<uchar>(_startPos_IJ[0],_startPos_IJ[1]) );
    ROS_INFO_STREAM( _goalPos_IJ[0] << " " << _goalPos_IJ[1] << " Pix : " << (int)_occupancyGridImage.at<uchar>(_goalPos_IJ[0],_goalPos_IJ[1]) );

    // pass the start and goal pixels to planning algorithm
    _LocalPlannerAStar->setStartAndGoalPixels(_startPos_IJ, _goalPos_IJ);
}




/* Converts an std::vector point into eigen::vector3d object */
template <typename T>
Eigen::Vector3d LocalPlanner::_getEigenVector3Point(const std::vector<T>& point){

    if(point.size() != 2)
    {
        ROS_ERROR( "LOCAL_PLANNER.cpp : Received a vector of size 3 when expected size was 2 \n" );
        exit(0); 
    }

    Eigen::Vector3d eigenPoint;
    eigenPoint << point[0] , point[1] , (T)1;
    return eigenPoint;
}




/* Converts an eigen::Vector3d point into an std::vector object */
template < typename T >
std::vector<T> LocalPlanner::_getSTDVectorPoint(Eigen::Vector3d& eigenPoint){

    std::vector<T> point{(T)eigenPoint[0], (T)eigenPoint[1]};
    return point;
}





/* Overloads / operator to perform vector / scalar division */
template<typename T1, typename T2>
std::vector<T1> operator/(const std::vector<T1>& vect, const T2& scalar){
    
    int size = vect.size();

    std::vector<T1> outputVector(size, (T2)0);
    
    for(int i=0; i<size; i++){
        outputVector[i] = vect[i] / scalar;
    }

    return outputVector;
}





/* Overloads * operator to perform vector * scalar multiplication */
template<typename T1, typename T2>
std::vector<T1> operator*(const std::vector<T1>& vect, const T2& scalar){
    
    int size = vect.size();

    std::vector<T1> outputVector(size, (T2)0);
    
    for(int i=0; i<size; i++){
        outputVector[i] = vect[i] * scalar;
    }

    return outputVector;
}




/* Initialize boundary points into pixel coordinates */
void LocalPlanner::_initializeStartAndGoalInPixel(){

    // start pixel
    _startPos_IJ.reserve(2);

    Eigen::Vector3d pixelStartPointInWorldFrame = _getEigenVector3Point(_startPos_XY/_costmapResolution);

    Eigen::Vector3d pixelStartPointInImageFrame = _convertFromWorldFrame(pixelStartPointInWorldFrame);       

    _startPos_IJ = _getSTDVectorPoint<int>(pixelStartPointInImageFrame);

    std::cout << " Start Coords : " << _startPos_IJ[0] << " " << _startPos_IJ[1] << "\n";


    // goal pixel
    _goalPos_IJ.reserve(2);

    Eigen::Vector3d pixelGoalPointInWorldFrame = _getEigenVector3Point(_goalPos_XY/_costmapResolution);

    Eigen::Vector3d pixelGoalPointInImageFrame = _convertFromWorldFrame(pixelGoalPointInWorldFrame);

    _goalPos_IJ = _getSTDVectorPoint<int>(pixelGoalPointInImageFrame);

    std::cout << " Goal Coords : " << _goalPos_IJ[0] << " " << _goalPos_IJ[1] << "\n";


    // checking if start and goal pixels are within bounds
    bool isStartPointTooCloseToObstacle = (int)_occupancyGridImage.at<uchar>(_startPos_IJ[0],_startPos_IJ[1])>10;
    bool isGoalPointTooCloseToObstacle = (int)_occupancyGridImage.at<uchar>(_goalPos_IJ[0],_goalPos_IJ[1])>10;

    if(isStartPointTooCloseToObstacle ){
        
        ROS_ERROR("The current start  position is very close to the obstacle, KUCH TO CHANGE KAR");
        exit(0);
    }
    if(isGoalPointTooCloseToObstacle ){
        
        ROS_ERROR("The current goal  position is very close to the obstacle, KUCH TO CHANGE KAR");
        exit(0);
    }

}





/* Converts the occupancy callback message into 2d CV::Mat based grid */
void LocalPlanner::_occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)  {

    // get grid parameters
    int size = (grid->data).size();


    int numRows = grid->info.height;
    int numCols = grid->info.width;


    // set member variables
    _costmapSize = numRows;
    _costmapOriginX = grid->info.origin.position.x;
    _costmapOriginY = grid->info.origin.position.y;
    _costmapResolution = grid->info.resolution;

    std::cout << "Origin X : " << _costmapOriginX <<"\n"; 
    std::cout << "Origin Y : " << _costmapOriginY <<"\n"; 

    // setup blank image
    _occupancyGridImage = cv::Mat(numRows, numCols, CV_8UC1, cv::Scalar(255));

   

    // fillup the image
    for (int k=0; k<size; k++){
        auto intensity = (grid->data[k]) * 255 / 100;
        _occupancyGridImage.at<uchar>(numCols - 1 - k % numCols , numRows - 1 - k / numCols) = intensity;
    }

    // cv::circle(mapImg, cv::Point(numCols - 1 - 147455 % numCols , numRows - 1 - 147455 / numCols), 2, cv::Scalar(0,0,255), -1);

    if(_b_resizeGrid)
        _resizeGrid();

    _b_isOccupancyGridImageInitialized = true;


    // initialize the transformation matrix from Gazebo's world to opencv image frame
    _initializeTransformationMatrix();


    // display the image
    // displayLocalGrid();
}





/* Loads the rosparams for local planner */
void LocalPlanner::_loadParams(){

    while(_LocalPlannerName=="unknown" && ros::ok() ){
        _nh.getParam("robot_info/planner_config/global_planner", _LocalPlannerName);
        _nh.getParam("robot_info/planner_config/global_planner_config/grid_connections", _gridConnections);
        _nh.getParam("robot_info/planner_config/global_planner_config/resize_grid", _b_resizeGrid);
        _nh.getParam("robot_info/planner_config/global_planner_config/grid_size", _gridSize);
        _nh.getParam("robot_info/planner_config/global_planner_config/obstacle_threshold", _obstacleThreshold);
        ros::Duration(0.5).sleep();
    }
}





/* Initialize the transformation matrix from Gazebo's world frame to opencv image frame */
void LocalPlanner::_initializeTransformationMatrix(){


    // origin with effect of resolution
    int worldOriginInPixelUnitsX = _costmapOriginX/_costmapResolution;
    int worldOriginInPixelUnitsY = _costmapOriginY/_costmapResolution;


    // Transform (in pixel coords) from world origin to Rviz origin (image's bottom right corner)
    Eigen::Matrix3d T_World_RVIZ ;
    T_World_RVIZ << 1 , 0, worldOriginInPixelUnitsX,
                    0,  1, worldOriginInPixelUnitsY, 
                    0,  0,                        1;


    // Transform (in pixel coords) from Rviz origin( image's bottom right corner) to OpenCV's image frame (top left corner with inverted axes)
    Eigen::Matrix3d T_RVIZ_Image;
    T_RVIZ_Image << -1 , 0, _costmapSize,
                     0, -1, _costmapSize,
                     0,   0,           1;


    // T02 = T01 * T12
    _T_World_Image = T_World_RVIZ * T_RVIZ_Image;


}




/* converts a point from openCV frame to world frame */
Eigen::Vector3d LocalPlanner::_convertToWorldFrame(Eigen::Vector3d pointInImageFrame){
    return _T_World_Image * pointInImageFrame;
}





/* Converts a point from world frame to  frame */
Eigen::Vector3d LocalPlanner::_convertFromWorldFrame(Eigen::Vector3d pointInWorldFrame){
    return _T_World_Image.inverse() * pointInWorldFrame;
}




/* Displays the local grid */
void LocalPlanner::displayLocalGrid(){

    // wait till callbacks are initialized
    while(!_b_isOccupancyGridImageInitialized && ros::ok()){
        ROS_WARN("OccupancyGrid Not Yet initialized\n");
        ros::Duration(1).sleep();
    }


    std::cout << "Image rows : " << _occupancyGridImage.rows << "\n";

    // error if image is empty
    if(!_occupancyGridImage.data){
        ROS_ERROR("Could not load the map image");
        return;
    }


    // display image
    cv::namedWindow(_windowName , cv::WINDOW_AUTOSIZE);
    cv::imshow(_windowName, _occupancyGridImage);
    auto k = cv::waitKey(0);                     // change the constant if you want to see image for long time 

    // destroy image windows
    if(k==27){
        cv::destroyAllWindows();
        exit(0);
    }

    // cv::imwrite("/home/kshah/local.jpg", _occupancyGridImage);
}




/* Calls planning algorithm's makePlan function and receives a vector<vector<int>> path*/
std::vector<std::vector<double>> LocalPlanner::makePlan(){

    std::vector<std::vector<int>> bestPath = _LocalPlannerAStar->makePlan();

    _extractBestPath(bestPath);

    return _bestPathLocal;
}




/* Converts the local path from pixel coords to meters */
void LocalPlanner::_extractBestPath(const std::vector<std::vector<int>>& bestPath){

    
    int size = bestPath.size();
    _bestPathLocal.reserve(size);


    for(int i=0; i<size; i++){
        // convert the vector<i,j> into eigen point & transform it into world frame from pixel frame
        Eigen::Vector3d bestEigenPointInPixelCoords = _convertToWorldFrame(
                                            _getEigenVector3Point(bestPath[i] / 1.0f ));


        // after getting it into req resolution, convert it into std::vector<double>(x,y) point in meters
        _bestPathLocal.emplace_back(
                        _getSTDVectorPoint<double>(
                            bestEigenPointInPixelCoords) * _costmapResolution );
    }

}





/* Resize grid if user resolution is different from RVIZ costmap resolution*/
void LocalPlanner::_resizeGrid(){
    auto line_no = __LINE__+2;
    try{
        cv::resize(_occupancyGridImage, _occupancyGridImage, cv::Size(_gridSize,_gridSize));
    }
    catch(std::exception& e){
        ROS_ERROR( "Error resizing the image");
        std::cerr << "Exception reference : " << e.what();
        std::cerr << "Error on line " << line_no << " of Local Planner \n\n";
    }

    // update the costmap resolution & costmap size
    _costmapResolution = _costmapSize * _costmapResolution / _gridSize;
    
    _costmapSize = _gridSize;        // as set by user in hmp_planner_params.yaml
}


int main(int argc, char** argv){

    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;

    LocalPlanner lp(nh);

    std::vector<double> goal{5.0f, 0.0f};
    // lp.setGoal(goal);

    // lp.makePlan();

    lp.displayLocalGrid();
}


/*
    API:
    
    constructor+ nh
    setGoal
    makePlan
    
*/