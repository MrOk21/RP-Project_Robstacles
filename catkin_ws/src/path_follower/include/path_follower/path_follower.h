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
  bool is_new_target;

  bool path_received;
  nav_msgs::Path received_path;

  nav_msgs::Path path;
  ros::Subscriber path_sub;
  ros::Subscriber initpose_sub;
  ros::Subscriber targetpose_sub;

  ros::Publisher initpose_pub; // To keep track of the poses that have been executed.

  int slow_rate = 5;
  int fast_rate = 10;

  void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void TargetPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
  void updateRobotPose(const nav_msgs::Path& path);
  void publishRobotPose(double x, double y, double theta);

  // Dynamically allocates memory, it takes on its own the management of a resource, and it does not 
  // require to delete that resource.
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster;
  
  ros::NodeHandle nh;
 

};

#endif  // PATH_FOLLOWER_H
