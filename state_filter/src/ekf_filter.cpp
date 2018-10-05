#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "StateFilter.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKF_Filter");
    ros::NodeHandle n("~");
    StateFilter<float,6,6> sf(n);
    ros::spin();
    return 0;
}