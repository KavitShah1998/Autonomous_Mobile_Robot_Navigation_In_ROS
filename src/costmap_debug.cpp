#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> // has shapes

void costmapCb(const nav_msgs::OccupancyGridConstPtr& grid){
    std::cout << " Hello World~\n";
    std::cout << " Map resolution:  " << grid->info.resolution << "\n";
    std::cout << " Map width: " << grid->info.width << "\n";
    std::cout << " Map height: " << grid->info.height << "\n";
    std::cout << " Map Load time: " << grid->info.map_load_time << "\n";
    std::cout << " Map Origin " << grid->info.origin.position.x << " " << grid->info.origin.position.y << "\n";
    std::cout << " Map data size: " << (grid->data).size() << "\n";

    int numRows = grid->info.height;
    int numCols = grid->info.width;
    int gridSize = (grid->data).size();

    cv::Mat mapImg = cv::Mat(numRows, numCols, CV_8UC3, cv::Scalar(255,255,255));

    for (int k=0; k<gridSize; k++){

        auto intensity = (grid->data[k]) * 255 / 100;
        cv::Vec3b color = cv::Vec3b(intensity,intensity,intensity);
        mapImg.at<cv::Vec3b>(numCols - 1 - k % numCols , numRows - 1 - k / numCols) = color;

        // if(k==0)
        //     cv::circle(mapImg, cv::Point(0,0), 2, cv::Scalar(0,0,255), -1);
    }

    cv::circle(mapImg, cv::Point(numCols - 1 - 147455 % numCols , numRows - 1 - 147455 / numCols), 2, cv::Scalar(0,0,255), -1);
    if(!mapImg.data){
        ROS_ERROR("Could not load the map image");
        return;
    }
    cv::namedWindow("Occupancy Costmap" , cv::WINDOW_AUTOSIZE);
    cv::imshow("Occupancy Costmap", mapImg);
    auto k = cv::waitKey(0);
    if(k==27){
        cv::destroyAllWindows();
        exit(0);
    }

}

int main(int argc, char ** argv){
    ros::init(argc, argv, "costmap_debug_node");

    ros::NodeHandle nh;

    ros::Subscriber sb = nh.subscribe("/move_base/global_costmap/costmap", 1, costmapCb);

    ros::spin();

    return 0;
}