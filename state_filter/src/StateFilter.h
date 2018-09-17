#ifndef SLAMMARKI_FILTER_H_
#define SLAMMARKI_FILTER_H_

#include <ros/ros.h>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include <ctime>
#include <opencv2/opencv.hpp>

class PipeEKF : public rgbd::ExtendedKalmanFilter<float, 4, 6>
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
    // mHZk << (fx * mXfk[0] / mXfk[2]) + Cx, (fy * mXfk[1] / mXfk[2]) + Cy, 1,
    //     (fx * mXfk[3] / mXfk[5]) + Cx, (fy * mXfk[4] / mXfk[5]) + Cy, 1;
  }
  void updateJh()
  {
    float fx = 726.429011;
    float fy = 721.683494;
    // mJh << fx / mXfk[0], 0, -fx * mXfk[0] / (mXfk[2] * mXfk[2]), 0, 0, 0,
    //     0, fy / mXfk[1], -fy * mXfk[1] / (mXfk[2] * mXfk[2]), 0, 0, 0,
    //     0, 0, 1, 0, 0, 0,
    //     0, 0, 0, fx / mXfk[3], 0, -fx * mXfk[3] / (mXfk[5] * mXfk[5]),
    //     0, 0, 0, 0, fy / mXfk[4], -fy * mXfk[4] / (mXfk[5] * mXfk[5]),
    //     0, 0, 0, 0, 0, 1;
  }
};

void centroidCallback(const ardrone_autonomy::Navdata imu){};
void imuCallback(const ardrone_autonomy::Navdata imu){};

class StateFilter
{

public:

  StateFilter(ros::NodeHandle &n) : nh_(n)
  {
    centroid_subscriber = n.subscribe("/ardrone/navdata", 10, centroidCallback);
    imu_subscriber = n.subscribe("/ardrone/navdata", 10, imuCallback);
    if (!mKalmanInitialized)
    {
      initializeKalmanFilter();
    }
    else
    {
      computeKalmanFilter();
    }
  }

  // Initialize EKF with first observation of centroid and altitude
  void initializeKalmanFilter();

  bool computeKalmanFilter();

public:
  ros::NodeHandle nh_;
  ros::Publisher pipe_pub_;
  ros::Subscriber centroid_subscriber;
  ros::Subscriber imu_subscriber;
  Eigen::Matrix<float, 3, 3> mIntrinsic;
  PipeEKF ekf;
  bool mKalmanFilter = true;
  bool mKalmanInitialized = false;
  std::vector<double> centroid;
  std::vector<double> p1;
};
#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_