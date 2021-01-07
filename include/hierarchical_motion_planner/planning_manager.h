//om 

# pragma once

# include <ros/ros.h>
# include <vector>
# include <memory>

# include "hierarchical_motion_planner/trajectory_executor.h"
# include "hierarchical_motion_planner/global_planner.h"



class PlanningManager{
    public:

        /**
         * \brief      initializes the class objects
         */
        PlanningManager(ros::NodeHandle &nh);

        
        /**
         * \brief      initializes the class objects
         */
        PlanningManager();

        /**
         * \brief      initializes the class objects
         */
        ~PlanningManager();

        /**
         * \brief      sets start and goal positions
         *
         * \param     start - start point as a 2 element vector 
         * \param     goal - goal point as a 2 element vector
         */
        void setStartAndGoal(std::vector<double> start, std::vector<double> goal);


        /**
         * \brief      sets start and goal positions
         */
        void execute();

    private:
        
        ros::NodeHandle _nh;

        std::unique_ptr<GlobalPlanner> _globalPlanner;
        std::unique_ptr<TrajectoryExecutor> _trajectoryController;

        std::vector<std::vector<double>> _path;


        /**
         * \brief      convert 2D vector of path into 1D vector of 
         *             geometry::msgs Point
         *  
         */
        std::vector<geometry_msgs::Point> _getTrajectoryFromPath(const std::vector<std::vector<double>>& path);

        /**
         * \brief      convert 1D vector of geometry::msgs Point 
         *             into 2D vector of path 
         */
        void _getPathFromTrajectory();

};