//om
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>  
#include <cmath>
#include <memory>


class PID_Base{

  public:

    /**
		 * \brief      Assigns pid gains 
     * 
     * \param     kp - p gain
     * \param     ki - i gain
     * \param     kd - d gain
		 */
    PID_Base(double kp, double kd, double ki);          // Class Constructor


    /**
		 * \brief      No function yet 
     * 
		 */
    PID_Base();                                       // Class Constructor


     /**
		 * \brief      Destructor
     * 
		 */
    ~PID_Base();                                      // Class Destructor


    /**
		 * \brief      Assigns pid gains 
     * 
     * \param     kp - p gain
     * \param     ki - i gain
     * \param     kd - d gain
		 */
    void setGains(double kp, double kd, double ki);               // passing p, i, d gains to the class members


    /**
		 * \brief      Computes the pid error
     * 
     * \param     error - error between current state
     *                    and goal state
     * 
		 */
    double getDrive(double error);                                // return the p i d drive as per the given error



  private:


    double _prevError{0.0f};                                    // storing prev error value
    double _iMax{300.0f}, _iMin{0.0f};                          // storing max and min values for integrator windup


    double _kp{0.0f}, _ki{0.0f}, _kd{0.0f};                  // p, i, d gains 
    double _pError{0.0f}, _iError{0.0f}, _dError{0.0f};      // p, i, d errors


    /**
		 * \brief      Computes the integtator windup
     *             to limit the integrator error
     *             from becoming astronomical 
     * 
     * \param      error - error between current state
     *             and goal state
		 */
    double _integratorWindup(double error);                              // returns i_error considering integrator windups


    /**
		 * \brief      Helper Function
     *             to display error 
     * 
		 */
    void _showError();                                                  // display the error  <Helper Function> 

};


class PID : public PID_Base{

  public:

    // /**
		//  * \brief      Initialize variables, set goal
    //  * 
    //  * \param      goal - defines the goal for the pid controller
		//  */
    // PID(geometry_msgs::Point goal);


      /**
		 * \brief      Initialize variables, set goal
     * 
     * \param      nh - accepts NodeHandle as an argument
		 */
    PID(ros::NodeHandle& nh);


    /**
		 * \brief      Initialize variables 
     * 
		 */
    PID();


    /**
		 * \brief      Frees up memory
     * 
		 */
    ~PID();


    /**
		 * \brief      Sets goal for robot 
     * 
     * \param      goal - defines the goal for the pid controller
		 */
    void setGoal(geometry_msgs::Point goal);


    /**
		 * \brief      Commands robot to move 
     * 
		 */
    bool execute();


  private:

    ros::NodeHandle _nh;
    ros::Subscriber _odomSubscriber;
    ros::Publisher  _velPublisher;
    // ros::Rate       _r(10);


    std::unique_ptr<PID_Base> _linearPID;
    std::unique_ptr<PID_Base> _angularPID;


    bool _b_isCollisionImminent{false};
    bool _b_reachedGoal{false};


    bool _b_setGoal{false};
    geometry_msgs::Point _goalPosition;
    double _goalOrientation;


    geometry_msgs::Point _currentPosition;
    double _currentOrientation;


    /**
		 * \brief      wraps angle between [-pi , pi]
     * 
     * \param      angle - angle in radians
		 */
    double _wrapAngle(double angle);


    /**
		 * \brief      computes distance between two points
     * 
     * \param      currentPoint, goalPoint
		 */
    double norm(geometry_msgs::Point& currPoint, geometry_msgs::Point& goalPoint);


    /**
		 * \brief      callback function for odom subscriber
     * 
     * \param      ptr - const ptr to Odometry datatype
		 */
    void _odomCallbackFunction(const nav_msgs::Odometry::ConstPtr& ptr);


    /**
		 * \brief      adjusts the robot velocity to avoid drifts 
     * 
     * \param      computedVelocity - velocity computed by the pid
     *             controller
		 */
    geometry_msgs::Twist _adjustVelocities(geometry_msgs::Twist& computedVelocity);
}; 




















