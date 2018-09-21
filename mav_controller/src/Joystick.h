#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <ctime>
#include <chrono>
#include "ardrone_autonomy/Navdata.h"

class Joystick
{
public:
  Joystick(ros::NodeHandle &n);

  void jsCallback(const ardrone_autonomy::Navdata imu);

private:
  ros::NodeHandle nh_;
  ros::Subscriber controller_subscriber;
};
#include "Joystick.inl"
#endif // JOYSTICK_H_