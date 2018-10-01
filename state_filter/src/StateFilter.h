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
    float cx = 400.5;
    float cy = 300.5;
    float Xm = mXfk[0];
    float Ym = mXfk[1];
    float Zm = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];
    float a = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float h = (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay)) / a;
    float xi = cx + fx * (Xm * (sin(ax) * sin(ay) * cos(az) - sin(az) * cos(ax)) + Ym * cos(ay) * cos(az) + Zm * (-sin(ax) * sin(az) - sin(ay) * cos(ax) * cos(az))) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay));
    float yi = cy + fy * (Xm * (sin(ax) * sin(ay) * sin(az) + cos(ax) * cos(az)) + Ym * sin(az) * cos(ay) + Zm * (sin(ax) * cos(az) - sin(ay) * sin(az) * cos(ax))) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay));
    mHZk << xi, yi, h, ax, ay, az;
  }
  void updateJh()
  {
    float fx = 674.3157444517138;
    float fy = 674.3157444517138;
    float cx = 400.5;
    float cy = 300.5;
    float Xm = mXfk[0];
    float Ym = mXfk[1];
    float Zm = mXfk[2];
    float ax = mXfk[3];
    float ay = mXfk[4];
    float az = mXfk[5];

    float landa = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);

    float a = fx * (sin(ax) * sin(ay) * cos(az) - sin(az) * cos(ax)) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay)) - fx * (Xm * (sin(ax) * sin(ay) * cos(az) - sin(az) * cos(ax)) + Ym * cos(ay) * cos(az) + Zm * (-sin(ax) * sin(az) - sin(ay) * cos(ax) * cos(az))) * sin(ax) * cos(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2);

    float b = fx * (Xm * (sin(ax) * sin(ay) * cos(az) - sin(az) * cos(ax)) + Ym * cos(ay) * cos(az) + Zm * (-sin(ax) * sin(az) - sin(ay) * cos(ax) * cos(az))) * sin(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2) + fx * cos(ay) * cos(az) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay));

    float c = fx * (-sin(ax) * sin(az) - sin(ay) * cos(ax) * cos(az)) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay)) + fx * (Xm * (sin(ax) * sin(ay) * cos(az) - sin(az) * cos(ax)) + Ym * cos(ay) * cos(az) + Zm * (-sin(ax) * sin(az) - sin(ay) * cos(ax) * cos(az))) * cos(ax) * cos(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2);

    float d = fy * (sin(ax) * sin(ay) * sin(az) + cos(ax) * cos(az)) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay)) - fy * (Xm * (sin(ax) * sin(ay) * sin(az) + cos(ax) * cos(az)) + Ym * sin(az) * cos(ay) + Zm * (sin(ax) * cos(az) - sin(ay) * sin(az) * cos(ax))) * sin(ax) * cos(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2);

    float e = fy * (Xm * (sin(ax) * sin(ay) * sin(az) + cos(ax) * cos(az)) + Ym * sin(az) * cos(ay) + Zm * (sin(ax) * cos(az) - sin(ay) * sin(az) * cos(ax))) * sin(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2) + fy * sin(az) * cos(ay) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay));

    float f = fy * (sin(ax) * cos(az) - sin(ay) * sin(az) * cos(ax)) / (Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay)) + fy * (Xm * (sin(ax) * sin(ay) * sin(az) + cos(ax) * cos(az)) + Ym * sin(az) * cos(ay) + Zm * (sin(ax) * cos(az) - sin(ay) * sin(az) * cos(ax))) * cos(ax) * cos(ay) / pow(Xm * sin(ax) * cos(ay) - Ym * sin(ay) - Zm * cos(ax) * cos(ay), 2);

    float g = sin(ax) * cos(ay) / landa;

    float h = -sin(ay) / landa;

    float i = -cos(ax) * cos(ay) / landa;

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