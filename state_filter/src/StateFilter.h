#ifndef SLAMMARKI_FILTER_H_
#define SLAMMARKI_FILTER_H_

#include <ros/ros.h>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
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
    float a = -((fx * cos(ax) * (1 / cos(az))) / (z));
    float b = (fx * cos(ax) * cos(ay)) * (x * (1 / cos(ay)) * (1 / cos(az)) - Cx) / (z * z);
    float c = -((fx * x * cos(ax) * tan(az) * (1 / cos(az))) / z);
    float d = -(fy / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay))));
    float e = (fy * cos(ax) * (Cy * cos(az) + y * (1 / cos(ax)) - Cy * sin(ay) * sin(az) * tan(ax))) / (z * z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)));
    float f = (fy * cos(ax) * (Cy * cos(az) + y * (1 / cos(ax)) - Cy * sin(ay) * sin(az) * tan(ax)) * ((1 / cos(ay)) * sin(az) + cos(az) * tan(ax) * tan(ay))) / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)) * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay))) -
              (fy * cos(ax) * ((-Cy) * sin(az) - Cy * cos(az) * sin(ay) * tan(ax))) / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)));
    float h = -(((1 / cos(ax)) * (1 / cos(ay))) / sqrt(1 + tan(ax) * tan(ax) + tan(ay) * tan(ay)));

    mHZk << a, 0, b, 0, 0, c,
        0, d, e, 0, 0, f,
        0, 0, h, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
  }
  void updateJh()
  {
    mJh << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
  }
};

class StateFilter
{
public:
  StateFilter(ros::NodeHandle &n);

  void centroidCallback(const ardrone_autonomy::Navdata imu);
  void IMUCallback(const ardrone_autonomy::Navdata imu);

  // Initialize EKF with first observation of centroid and altitude
  void initializeKalmanFilter();

  bool computeKalmanFilter(float _incT);

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
  float altitude = 0;
  bool newData = false;
  std::chrono::steady_clock::time_point t0;
};
#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_