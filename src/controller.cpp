# include "hierarchical_motion_planner/controller.h"

PID_Base :: PID_Base() : _kp(0.0f) , _ki(0.0f) , _kd(0.0f){
  std::cout << "PID_Base Default C'tor \n";
}


PID_Base :: PID_Base(double kp, double kd, double ki) : _kp(kp) , _ki(ki) , _kd(kd){
  std::cout << "PID_Base C'tor \n";

}

PID_Base :: ~PID_Base(){ }


void PID_Base :: setGains( double kp, double kd, double ki) {
  _kp = kp ;
  _ki = ki ; 
  _kd = kd ; 

} 


double PID_Base :: getDrive( double error){

  _pError = error;
  _iError = _integratorWindup(error);
  _dError = error - _prevError;
  double net_drive = _kp * error + _kd * (error - _prevError) + _ki * _integratorWindup(error);
  _prevError = error;

  return net_drive;

}


double PID_Base :: _integratorWindup(double error){
  
  if(_iError + error > _iMax)
    return _iMax;

  else if(_iError + error < _iMin)
    return _iMin;

  else
    return _iError + error;

}

void PID_Base :: _showError(){

  std::cout << " Proportional Error = " << _pError << "\n";
  std::cout << " Integrator Error = " << _iError << "\n";
  std::cout << " Derivative Error = " << _dError << "\n";
}







// PID :: PID( geometry_msgs::Point goal ) : _goalPosition(goal) , _b_setGoal(true){

//   std::cout << "PID C'tor \n";


//   _linearPID  = std::make_unique<PID_Base>( 0.0849 , 0.14 , 0.0011 );
//   _angularPID = std::make_unique<PID_Base>( 0.6519  , 0.1929, 0.0013 );

//   _velPublisher   = _nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
//   _odomSubscriber = _nh.subscribe<nav_msgs::Odometry> ("/odom" , 10, &PID::_odomCallbackFunction, this);

// } 


PID :: PID( ros::NodeHandle& nh) : _nh(nh){

  std::cout << "PID C'tor \n";


  _linearPID  = std::make_unique<PID_Base>( 0.0849 , 0.14 , 0.0011 );
  _angularPID = std::make_unique<PID_Base>( 0.6519  , 0.1929, 0.0013 );

  _velPublisher   = _nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
  _odomSubscriber = _nh.subscribe<nav_msgs::Odometry> ("/odom" , 10, &PID::_odomCallbackFunction, this);

} 


PID :: PID() {

  std::cout << "PID Default C'tor \n";


  _linearPID  = std::make_unique<PID_Base>( 0.029 , 0.0018 , 0.0025 );
  _angularPID = std::make_unique<PID_Base>( 0.03  , 0.0009 , 0.0002 );

  _velPublisher   = _nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
  _odomSubscriber = _nh.subscribe<nav_msgs::Odometry> ("/odom" , 10, &PID::_odomCallbackFunction, this);

} 


PID :: ~PID() {

  std::cout << "PID Default D'tor \n";

  geometry_msgs::Twist stopVelocity;

  int count{0};
  while(count++<10){
    _velPublisher.publish(stopVelocity);
    ros::Duration(0.1).sleep();
  }

} 


void PID :: setGoal(geometry_msgs::Point goal){
  _goalPosition = goal;
  _b_setGoal = true;
  _b_reachedGoal = false;
}


void PID :: _odomCallbackFunction ( const nav_msgs::Odometry::ConstPtr& ptr){

  _currentPosition = ptr->pose.pose.position;

  tf::Quaternion q( ptr->pose.pose.orientation.x,
                    ptr->pose.pose.orientation.y,
                    ptr->pose.pose.orientation.z,
                    ptr->pose.pose.orientation.w );

  tf::Matrix3x3 mat(q);

  double R,P,Y;

  mat.getRPY(R,P,Y);

  _currentOrientation = _wrapAngle( Y );

  _goalOrientation = _wrapAngle( atan2(_goalPosition.y - _currentPosition.y  ,  _goalPosition.x - _currentPosition.x) );

  _b_reachedGoal = norm(_currentPosition, _goalPosition) < 0.2;
}


double PID :: _wrapAngle(double angle){
  if(angle > M_PI)
    angle -= 2 * M_PI;
  else if(angle < -M_PI)
    angle += 2 * M_PI;

  return angle;
}


double PID :: norm(geometry_msgs::Point& currPoint, geometry_msgs::Point& goalPoint){

  return sqrt ( pow(goalPoint.x - currPoint.x,2)  + pow(goalPoint.y - currPoint.y,2));
}


bool PID :: execute(){

  if(!_b_setGoal){
    ROS_ERROR(" Controller Error : Robot Goal not set \n");
    return false;
  }
  std::cout << "\n Executing motion to goal: (" << _goalPosition.x << " , " << _goalPosition.y << ")\n";

  geometry_msgs::Twist robotVelocity;
  while( ros::ok() && !_b_isCollisionImminent && !_b_reachedGoal ){

    // get linear drive
    robotVelocity.linear.x = _linearPID->getDrive(norm( _currentPosition , _goalPosition ));
    
    // get angular drive
    robotVelocity.angular.z = _angularPID->getDrive(_wrapAngle(_goalOrientation - _currentOrientation));


    // std::cout << "Linear = " << robotVelocity.linear.x;
    // std::cout << " Angular = " << robotVelocity.angular.z  << " Error = " << _goalOrientation - _currentOrientation << "\n";
    _velPublisher.publish(_adjustVelocities(robotVelocity));
    

    ros::spinOnce();
    ros::Duration(0.1).sleep();


    // check if collision detected by obstacle_checker_node
    _nh.getParam("/hierarchical_planner/is_robot_colliding", _b_isCollisionImminent);
  }

  return _b_reachedGoal;
}


geometry_msgs::Twist PID :: _adjustVelocities(geometry_msgs::Twist& computedVelocity){

  return computedVelocity;
}