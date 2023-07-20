#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>



using namespace std;
using namespace cv;

class OccupancyGridUtils{

    public:
        // public variables
        double res; // map resolution
        int h;      // map height
        int w;      // map width
        double x;   // x coordinate of the origin
        double y;   // y coordinate of the origin
        double theta;  // respective orientation value

        // public functions
        void GetParams(const nav_msgs::OccupancyGridConstPtr& Occ_Grid); // Initialize main params (resolution, h, w and so on...)
        void Map2ImageTransform(const Point2d& map_point, Point& image_point);
        void Image2MapTransform(const Point& image_point, Point2d& map_point);

    private:
        // private variables
        Mat Rot;
        Mat t;


};





#endif // OCCUPANCY_GRID_H