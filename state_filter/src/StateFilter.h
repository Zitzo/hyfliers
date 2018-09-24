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
    float xi = -((fx * cos(ax) * cos(ay) * (x * (1 / cos(ay)) * (1 / cos(az)) - Cx)) / z);
    float yi = -((fy * cos(ax) * (-(Cy * tan(ax) * sin(ay) * sin(az)) + y * (1 / cos(ax)) + Cy * cos(az))) / (z * (tan(ax) * tan(ay) * sin(az) - (1 / cos(ay)) * cos(az))));
    float h = -(z * (1/cos(ax)) * (1/cos(ay))) / (sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1));
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
    float a = -((fx * cos(ax) * (1 / cos(az))) / (z));
    float b = (fx * cos(ax) * cos(ay)) * (x * (1 / cos(ay)) * (1 / cos(az)) - Cx) / (z * z);
    float c = -((fx * x * cos(ax) * tan(az) * (1 / cos(az))) / z);
    float d = -(fy / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay))));
    float e = (fy * cos(ax) * (Cy * cos(az) + y * (1 / cos(ax)) - Cy * sin(ay) * sin(az) * tan(ax))) / (z * z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)));
    float f = (fy * cos(ax) * (Cy * cos(az) + y * (1 / cos(ax)) - Cy * sin(ay) * sin(az) * tan(ax)) * ((1 / cos(ay)) * sin(az) + cos(az) * tan(ax) * tan(ay))) / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)) * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay))) -
              (fy * cos(ax) * ((-Cy) * sin(az) - Cy * cos(az) * sin(ay) * tan(ax))) / (z * ((-cos(az)) * (1 / cos(ay)) + sin(az) * tan(ax) * tan(ay)));
    float h = -(((1 / cos(ax)) * (1 / cos(ay))) / sqrt(1 + tan(ax) * tan(ax) + tan(ay) * tan(ay)));

    mJh << a, 0, b, 0, 0, c,
        0, d, e, 0, 0, f,
        0, 0, h, 0, 0, 0,
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