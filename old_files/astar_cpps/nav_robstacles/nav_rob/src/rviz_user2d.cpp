#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
 
using namespace std;
 
// Initialize ROS publishers, //This node is subscribed to rviz, and get infos about goal and init position given by the user. Then they are published, with two publishers. The following one.
ros::Publisher pub;
ros::Publisher pub2;
 
// Take move_base_simple/goal as input and publish goal_2d
void handle_goal(const geometry_msgs::PoseStamped &goal) {  //This node get the goal information from rviz (user selected(this node is subscribed to it)).
  geometry_msgs::PoseStamped rpyGoal; //Pose we want to publish for the goal.
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = goal.header.stamp;
  rpyGoal.pose.position.x = goal.pose.position.x;
  rpyGoal.pose.position.y = goal.pose.position.y;
  rpyGoal.pose.position.z = 0;
  tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = yaw;
  rpyGoal.pose.orientation.w = 0;
  pub.publish(rpyGoal);
}
 
// Take initialpose as input and publish initial_2d
void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &pose) {
  geometry_msgs::PoseStamped rpyPose; //Pose we want to publish
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = pose.header.stamp;
  rpyPose.pose.position.x = pose.pose.pose.position.x;
  rpyPose.pose.position.y = pose.pose.pose.position.y;
  rpyPose.pose.position.z = 0;
  tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyPose.pose.orientation.x = 0;
  rpyPose.pose.orientation.y = 0;
  rpyPose.pose.orientation.z = yaw;
  rpyPose.pose.orientation.w = 0;
  pub2.publish(rpyPose);
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_click_to_2d");
  ros::NodeHandle node;
  pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 0); //This topic is referred for example to the path planner....
  pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 0); //This topic is useful for the odometry node....
  ros::Subscriber sub = node.subscribe("move_base_simple/goal", 0, handle_goal); //As we get information from the rviz, either for the goal pose, either for the init pose
  ros::Subscriber sub2 = node.subscribe("initialpose", 0, handle_initial_pose);  //we publish it to the other topics 
  ros::Rate loop_rate(10);
  while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
  }
  return 0;
}
