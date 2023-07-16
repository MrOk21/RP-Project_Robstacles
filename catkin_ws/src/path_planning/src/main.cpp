#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "path_planning/occupancy_grid.h"






using namespace cv;
using namespace std;

OccupancyGridUtils occgrid_params;
nav_msgs::OccupancyGrid occgrid_mask;


// Global Variables
bool found_map;
bool found_start;
bool found_target;
bool start_plan;

int rate;

ros::Subscriber map_sub;


// Callbacks

void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Get parameters
    cout<<"Message is in the house"<<endl;
    occgrid_params.GetParams(msg);
    cout<<"Message is in the house2"<<endl;

    // Compute the map obj
    int h = occgrid_params.h;
    int w = occgrid_params.w;
    int occupancy_prob;

    // h x w matrix
    Mat Map(h, w, CV_8UC1);
    cout<<"Message is in the house3"<<endl;
    
    for(int i=0;i<h;i++)
    {
        for(int j = 0; j < w; j++)
        {
            // iterate over the data points of the map
            occupancy_prob = msg->data[i * w + j];
            // If OccProb is less than 0
            occupancy_prob = (occupancy_prob < 0) ? 100 : occupancy_prob; // set Unknown to 0
            // The origin of the Occ_Grid is on the bottom left corner of the map
            Map.at<uchar>(h-i-1, j) = 255 - round(occupancy_prob * 255.0 / 100.0);
            

        }
    }

    

    // Flags
    found_map = true;
    found_start = false;
    found_target = false;


}


// Main
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ROS_INFO("Start astar node!\n");

    // Initial variables
    found_map = false;
    found_start = false;
    found_target = false;
    start_plan = false;
    // Set ros rate to 10 Hz
    rate = 10;

    // Parameter

    // Subscribe topics
    map_sub = nh.subscribe("map", 10, MapCallback);

    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        if (found_map)
        {
            cout<<"Found MAP!!!!"<<endl;
        }
        // else
        //     {
        //     cout<<"MAP was not found!!!!"<<endl;

        //     }
        ros::spinOnce();  // 
        loop_rate.sleep();  // 
    }
}