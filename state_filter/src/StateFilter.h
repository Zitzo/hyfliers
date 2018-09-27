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
//template <typename Type_, int D1_, int D2_>
class PipeEKF : public rgbd::ExtendedKalmanFilter<float, 6, 6>
{
public:
  // ExtendedKalmanFilter(Eigen::Matrix<float, 3, 3> _intrinsic)
  // {
  //   mIntrinsic = _intrinsic;
  // }

protected:
  void updateJf(const double _incT)
  {
    mJf.setIdentity();
    //mJf.block<2, 2>(0, 2) = Eigen::Matrix<float, 2, 2>::Identity() * _incT;
  }
  void updateHZk()
  {
    float fx = 674.3157444517138;
    float fy = 674.3157444517138;
    float Cx = 400.5;
    float Cy = 300.5;
    float x = mXfk[0];
    float y = mXfk[1];
    float z = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];
    float a = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float h = (-z * cos(ax) * cos(ay) - y * (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) + x * (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az))) / a;
    float xi = (-y * cos(ay) * cos(az) + z * sin(ay) - x * cos(ay) * sin(az)) * fx / (h * a) + Cx;
    float yi = (-z * cos(ay) * sin(ax) + y * (-cos(az) * sin(ax) * sin(ay) + cos(ax) * sin(az)) + x * (-cos(ax) * cos(az) - sin(ax) * sin(ay) * sin(az))) * fx / (h * a) + Cy;
    mHZk << xi, yi, h, ax, ay, az;
  }
  void updateJh()
  {
    float fx = 674.3157444517138;
    float fy = 674.3157444517138;
    float Cx = 400.5;
    float Cy = 300.5;
    float x = mXfk[0];
    float y = mXfk[1];
    float z = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];
    float landa = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float altitude = (-z * cos(ax) * cos(ay) + x * (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) + y * (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az))) / landa;
    float a = -fx * cos(ay) * sin(az) / (landa * altitude);
    float b = -fx * cos(ay) * cos(az) / (landa * altitude);
    float c = fx * sin(ay) / (landa * altitude);
    float d = fy * (-cos(ax) * cos(az) - sin(ax) * sin(ay) * sin(az)) / (landa * altitude);
    float e = fy * (-cos(az) * sin(ax) * sin(ay) + cos(ax) * sin(az)) / (landa * altitude);
    float f = -fy * (cos(ay) * sin(ax)) / (landa * altitude);
    float g = (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az)) / landa;
    float h = (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) / landa;
    float i = -(cos(ax) * cos(ay)) / landa;

    mJh << a, b, c, 0, 0, 0,
        d, e, f, 0, 0, 0,
        g, h, i, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
  }

public:
  Eigen::Matrix<float, 3, 3> mIntrinsic;
};

struct Observation
{
  float altitude;
  float xi;
  float yi;
  Eigen::Quaternionf quat;
  std::chrono::steady_clock::time_point time;
};

template <typename Type_, int D1_, int D2_>
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
  ros::Publisher no_Filtered_pub;
  ros::Subscriber pipe_subscriber;
  Eigen::Matrix<float, 3, 3> mIntrinsic;
  PipeEKF ekf;
  bool mKalmanFilter = true;
  bool mKalmanInitialized = false;
  Observation mLastObservation;
  std::chrono::steady_clock::time_point t0;
};
#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_