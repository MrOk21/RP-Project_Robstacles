#include "path_planning/occupancy_grid.h"


void OccupancyGridUtils::GetParams(const nav_msgs::OccupancyGridConstPtr& msg_)
{

    // Parameters
    res = msg_->info.resolution;
    h = msg_->info.height;
    w = msg_->info.width;
    x = msg_->info.origin.position.x;
    y = msg_->info.origin.position.y;

    // quaternion computed from the orientation
    tf::Quaternion quat_angles(msg_->info.origin.orientation.x, msg_->info.origin.orientation.y, msg_->info.origin.orientation.z, msg_->info.origin.orientation.w);

    // Retrieve roll, pitch, yaw
    double roll, pitch, yaw;

    tf::Matrix3x3(quat_angles).getRPY(roll, pitch, yaw);
    theta = yaw;

    // CV_64FC1: 64-bit floating-point single-channel matrix
    Rot = cv::Mat::eye(2, 2, CV_64FC1);

    // Include the resolution of the map to compute the world frame values.
    // In our case the orientation and translation between the occupancy grid and the world frame are null.
    // Anyway there should be a conversion due to the resolution of the map.
    Rot.at<double>(0, 0) = res * cos(theta);
    Rot.at<double>(0, 1) = -res * sin(theta);
    Rot.at<double>(1, 0) = res * sin(theta);
    Rot.at<double>(1, 1) = res * cos(theta);

    t = cv::Mat(cv::Vec2d(x, y), CV_64FC1);
}

void OccupancyGridUtils::Map2ImageTransform(const Point2d& _map_point, Point& _image_point)
{

    cv::Mat wf_point_cv = cv::Mat(cv::Vec2d(_map_point.x, _map_point.y), CV_64FC1);

    // from world frame to occupancy grid
    cv::Mat oc_grid_point = Rot.inv() * (wf_point_cv - t);

    // from occupancy grid to image plane
    _image_point.x = round(oc_grid_point.at<double>(0, 0));
    _image_point.y = h - 1 - round(oc_grid_point.at<double>(1, 0));


}

void OccupancyGridUtils::Image2MapTransform(const Point& _image_point, Point2d& _map_point)
{
    // Image RF and Map RF share the same x-axis 
    // from image RF to Occupancy grid RF
    cv::Mat oc_grid_point = cv::Mat(cv::Vec2d(_image_point.x, h - 1 - _image_point.y), CV_64FC1);
    // from Occupancy Grid RF to World RF
    cv::Mat wf_point_cv = Rot * oc_grid_point + t;
    _map_point.x = wf_point_cv.at<double>(0, 0);
    _map_point.y = wf_point_cv.at<double>(1, 0);
}