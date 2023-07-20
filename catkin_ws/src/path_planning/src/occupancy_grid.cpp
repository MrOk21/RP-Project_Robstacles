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

    Rot.at<double>(0, 0) = res * cos(theta);
    Rot.at<double>(0, 1) = -res * sin(theta);
    Rot.at<double>(1, 0) = res * sin(theta);
    Rot.at<double>(1, 1) = res * cos(theta);

    t = cv::Mat(cv::Vec2d(x, y), CV_64FC1);
}

void OccupancyGridUtils::Map2ImageTransform(const Point2d& _map_point, Point& _image_point)
{

    cv::Mat image_point_cv = cv::Mat(cv::Vec2d(_map_point.x, _map_point.y), CV_64FC1);
    image_point_cv = Rot.inv() * (image_point_cv - t);

    _image_point.x = round(image_point_cv.at<double>(0, 0));
    _image_point.y = h - 1 - round(image_point_cv.at<double>(1, 0));


}

void OccupancyGridUtils::Image2MapTransform(const Point& _image_point, Point2d& _map_point)
{
    cv::Mat map_point_cv = cv::Mat(cv::Vec2d(_image_point.x, h - 1 - _image_point.y), CV_64FC1);
    map_point_cv = Rot * map_point_cv + t;
    _map_point.x = map_point_cv.at<double>(0, 0);
    _map_point.y = map_point_cv.at<double>(1, 0);
}