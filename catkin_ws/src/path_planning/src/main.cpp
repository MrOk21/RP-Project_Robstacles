#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "path_planning/occupancy_grid.h"
#include "path_planning/astar.h"







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


path_planning::Astar_Mods mods;
path_planning::Astar astar_planner;
nav_msgs::Path path_msg;
Point start_point, target_point;


// Ros Nodes
ros::Subscriber map_sub;
ros::Subscriber initpose_sub;
ros::Subscriber goalpose_sub;

ros::Publisher path_pub;


// Callbacks
void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d map_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    // start_point is transformed into an integer and converted into an image point
    occgrid_params.Map2ImageTransform(map_point, start_point);

    // Set flag
    found_start = true;
    ROS_INFO("INITIAL POSE have been found!");

    if(found_map && found_start && found_target)
    {
        start_plan = true;
    }
}

void TargetPointCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d map_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    occgrid_params.Map2ImageTransform(map_point, target_point);

    // Set flag
    found_target = true;
    if(found_map && found_start && found_target)
    {
        start_plan = true;
    }
    ROS_INFO("TARGET POSE have been found!");


}
void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Get parameters
    occgrid_params.GetParams(msg);

    // Compute the map obj
    int h = occgrid_params.h;
    int w = occgrid_params.w;
    int occupancy_prob;

    // h x w matrix
    Mat Map(h, w, CV_8UC1);
    
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

    // Initial Astar
    Mat Mask;
    
    astar_planner.InitAstar(Map, Mask, mods);
    // Publish Mask
    occgrid_mask.header.stamp = ros::Time::now();
    occgrid_mask.header.frame_id = "map";
    occgrid_mask.info = msg->info;
    occgrid_mask.data.clear();
    
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            occupancy_prob = Mask.at<uchar>(h-i-1, j) * 255;
            occgrid_mask.data.push_back(occupancy_prob);
        }
    }

    // Flags
    found_map = true;
    found_start = false;
    found_target = false;
    ROS_INFO("Map and Obstacle maps have been loaded!");


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
    mods.tolerance = 5;
    mods.Eucl_Heuristics = false; //false if heuristic is Mahnattan distance, otherwise is the Euclidean.

    // Parameter

    // Subscribe topics
    map_sub = nh.subscribe("map", 10, MapCallback);
    initpose_sub = nh.subscribe("initialpose", 10, StartPointCallback);
    goalpose_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointCallback);

    // Advertise topic
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);

    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        if (start_plan)
        {
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> waypoints;
            astar_planner.PathPlanning(start_point, target_point, waypoints); // PathList is referenced
            cout << waypoints << endl;
            if(!waypoints.empty())
            {
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";
                path_msg.poses.clear(); // path is a global variable, which must be cleared before filling it
                for(int i = 0; i < waypoints.size(); i++)
                {
                    Point2d map_point;
                    occgrid_params.Image2MapTransform(waypoints[i], map_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = map_point.x;
                    pose_stamped.pose.position.y = map_point.y;
                    pose_stamped.pose.position.z = 0;
                    path_msg.poses.push_back(pose_stamped);
                    // path is a container of poses.
                }

                path_pub.publish(path_msg);
               
                double end_time = ros::Time::now().toSec();

                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_plan = false;
        }
        // Look for the callbacks
        ros::spinOnce();  
        loop_rate.sleep();  
    }
}