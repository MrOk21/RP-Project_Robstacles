#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <ros/publisher.h>

class PathFollowerUtils
{
public:
  PathFollowerUtils();

private:
  bool is_new_initpose;
  bool path_received;
  nav_msgs::Path received_path;

  nav_msgs::Path path;
  ros::Subscriber path_sub;
  ros::Subscriber initpose_sub;
  void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
  void updateRobotPose(const nav_msgs::Path& path);
  void publishRobotPose(double x, double y, double theta);

  // Dynamically allocates memory, it takes on its own the management of a resource, and it does not 
  // require to delete that resource.
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster;
  
  ros::NodeHandle nh;
 

};

#endif  // PATH_FOLLOWER_H