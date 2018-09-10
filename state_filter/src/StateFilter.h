#ifndef SLAMMARKI_FILTER_H_
#define SLAMMARKI_FILTER_H_

#include <ros/ros.h>
// #include "geometry_msgs/Twist.h"

template <typename Type_, int D1_, int D2_>
class StateFilter
{
  public:
    StateFilter(ros::NodeHandle &n) : nh_(n), it_(nh_){};

    

  public:
    ros::NodeHandle nh_;
    ros::Publisher pipe_pub_;
    ros::Subscriber alt_sub_;
};

#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_