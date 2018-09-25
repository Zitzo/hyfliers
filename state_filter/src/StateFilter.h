#ifndef SLAMMARKI_FILTER_H_
#define SLAMMARKI_FILTER_H_

#include <ros/ros.h>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64.h"
#include <ctime>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <math.h>

class PipeEKF : public rgbd::ExtendedKalmanFilter<float, 6, 6>
{
protected:
  void updateJf(const double _incT)
  {
    mJf.setIdentity();
    //mJf.block<2, 2>(0, 2) = Eigen::Matrix<float, 2, 2>::Identity() * _incT;
  }
  void updateHZk()
  {
    float fx = 726.429011;
    float fy = 721.683494;
    float Cx = 283.809411;
    float Cy = 209.109682;
    float x = mXfk[0];
    float y = mXfk[1];
    float z = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];
    float xi = x*fx/z+Cx;
    float yi = y*fy/z+Cy;
    float h = z / sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    mHZk << xi, yi, h, ax, ay, az;
  }
  void updateJh()
  {
    float fx = 726.429011;
    float fy = 721.683494;
    float Cx = 283.809411;
    float Cy = 209.109682;
    float x = mXfk[0];
    float y = mXfk[1];
    float z = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];
    float a = fx/z;
    float b = -fx*x/(z*z);
    float c = fy/z;
    float d = -fy*y/(z*z);
    float e = 1/ sqrt(1 + tan(ax) * tan(ax) + tan(ay) * tan(ay));

    mJh << a, 0, b, 0, 0, 0,
        0, c, d, 0, 0, 0,
        0, 0, e, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
  }
};

struct Observation
{
  float altitude;
  float xi;
  float yi;
  Eigen::Quaternionf quat;
  std::chrono::steady_clock::time_point time;
};

class StateFilter
{
public:
  StateFilter(ros::NodeHandle &n);

  void pipeDetectionCallback(const geometry_msgs::PoseStamped msg);

  // Initialize EKF with first observation of centroid and altitude
  void initializeKalmanFilter();

  bool computeKalmanFilter(float _incT);

public:
  ros::NodeHandle nh_;
  ros::Publisher filtered_pub;
  ros::Subscriber pipe_subscriber;
  Eigen::Matrix<float, 3, 3> mIntrinsic;
  PipeEKF ekf;
  bool mKalmanFilter = true;
  bool mKalmanInitialized = false;
  std::vector<double> centroid;
  Observation mLastObservation;
  std::chrono::steady_clock::time_point t0;
};
#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_