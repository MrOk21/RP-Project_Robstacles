#include "path_follower/path_follower.h"


PathFollowerUtils::PathFollowerUtils()
{
  // Initialized in the main


  tf_broadcaster.reset(new tf::TransformBroadcaster()); // new TransformBroadcaster in the dynamic memory

  // Subscribe to the "/nav_path" topic
  path_sub = nh.subscribe<nav_msgs::Path>("/nav_path", 1, &PathFollowerUtils::pathCallback, this);

  // Main loop to continuously update the robot pose from RViz
  ros::Rate rate(10);  // 10 Hz

  while (ros::ok())
  {
    // Look for path callbacks
    ros::spinOnce();
    rate.sleep();
  }
}


void PathFollowerUtils::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  // Process the received path
  std::vector<geometry_msgs::PoseStamped> waypoints = path_msg->poses;

  ROS_INFO("Received path with %lu waypoints", waypoints.size());

  // Pass the path to the updateRobotPose function
  updateRobotPose(*path_msg);
}

void PathFollowerUtils::updateRobotPose(const nav_msgs::Path& path)
{
 
  ros::Rate rate(1);
  for (const auto& pose : path.poses)
    {
      
      publishRobotPose(pose.pose.position.x, pose.pose.position.y, 0.0);
      rate.sleep();
    
    }

}


void PathFollowerUtils::publishRobotPose(double x, double y, double theta)
{
  // Publish the updated robot pose as a tf transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.1)); 
  tf::Quaternion quaternion;
  quaternion.setRPY(0, 0, theta);
  transform.setRotation(quaternion);
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

}