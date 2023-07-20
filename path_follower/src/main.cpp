#include "path_follower/path_follower.h"


int main(int argc, char** argv)

{

    ros::init(argc, argv, "path_follower_node");

    PathFollowerUtils path_follower;

    ros::spin();

    return 0;

}