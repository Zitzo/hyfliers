#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "StateFilter.h"
#include <iostream>

int main(int argc, char **argv)
{
    ROS_INFO("Initializating state filter");
    ros::init(argc, argv, "State_Filter");
    ros::NodeHandle n("~");
    StateFilter sf(n);
    ros::spin();
    return 0;
}



