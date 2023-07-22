#include "path_follower/path_follower.h"
using namespace std;


PathFollowerUtils::PathFollowerUtils()
{
    // Initialization to smart pointer, in order to dynamically allocate memory easily and avoiding doing delete.
    tf_broadcaster.reset(new tf::TransformBroadcaster());

    // Subscribe to the "/nav_path" topic
    // 10 Hz to get the path subscription
    // Check for the path
    path_sub = nh.subscribe<nav_msgs::Path>("/nav_path", slow_rate, &PathFollowerUtils::pathCallback, this);


    // Check if a new initpose or target pose has been published in order to find a new path.
    initpose_sub = nh.subscribe("initialpose", fast_rate, &PathFollowerUtils::StartPointCallback, this);
    targetpose_sub = nh.subscribe("move_base_simple/goal", fast_rate, &PathFollowerUtils::TargetPointCallback, this);


    initpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", fast_rate);
    // Main loop to continuously update the robot pose from RViz
    ros::Rate rate(fast_rate); // 10 Hz

    while (ros::ok())
    {
        // Look for path and initial pose callbacks
        ros::spinOnce();

        if (path_received) // Whenever a path to follow is generated the node will follow it.
        {   
            is_new_initpose = false;
            is_new_target = false;
            updateRobotPose(received_path); // If the initialpose changes, the new path will be executed
            continue;
        }
        
        rate.sleep();
    }
}

void PathFollowerUtils::StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    const geometry_msgs::Pose &pose = msg->pose.pose;

    double x = pose.position.x;
    double y = pose.position.y;

    double theta = tf::getYaw(pose.orientation);

    // Publish new pose
    publishRobotPose(x, y, theta);
    // broadcasting that a new pose has been set.
    is_new_initpose = true;
    
}

void PathFollowerUtils::TargetPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    is_new_target = true;
}

void PathFollowerUtils::pathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
    // Process the received path
    std::vector<geometry_msgs::PoseStamped> waypoints = path_msg->poses;

    ROS_INFO("Received path with %lu waypoints", waypoints.size());

    // Store the received path and set the flag
    received_path = *path_msg; // Path to follow.
    path_received = true;  // Flag to broadcast that a new path must be followed.

}

void PathFollowerUtils::updateRobotPose(const nav_msgs::Path &path)
{
    ros::Rate rate(slow_rate); // 5 Hz to update the robot pose along the path.
    // Go through the received path and publish the robot's pose for the next waypoint
    path_received = false; // This will be set true when a new path is generated.
    for (const auto &pose : path.poses)
    {
        ros::spinOnce(); // Look for is_new_initpose callbacks.
        if (is_new_initpose)
            {
                is_new_initpose = false;
                break;
            }
        else if (is_new_target) // Look for new goal poses.
            {   
                // Pose message for the new starting point
                geometry_msgs::PoseWithCovarianceStamped pose_msg;


                pose_msg.header.frame_id = "map";  // Map frame
                pose_msg.pose.pose.position.x = pose.pose.position.x;
                pose_msg.pose.pose.position.y = pose.pose.position.y;
                pose_msg.pose.pose.position.z = 0.1;
                pose_msg.pose.pose.orientation.x = 0.0;
                pose_msg.pose.pose.orientation.y = 0.0;
                pose_msg.pose.pose.orientation.z = 0.0;
                pose_msg.pose.pose.orientation.w = 1.0; // No rotation specified

                initpose_pub.publish(pose_msg); // Publish new message of the initial position
                is_new_target = false;
                break;
            }

        // Publish each orientation in path.poses.
        publishRobotPose(pose.pose.position.x, pose.pose.position.y, 0.0);
        
        rate.sleep();
    }
}



void PathFollowerUtils::publishRobotPose(double x, double y, double theta)
{
    // Publish the updated robot pose as a tf transform; Thise can be made more complex for more complex shapes.
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.1)); 
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);
    transform.setRotation(quaternion);
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}