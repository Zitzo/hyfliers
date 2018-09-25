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
    float a = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float h = (-z * cos(ax) * cos(ay) + x * (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) + y * (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az))) / a;
    float xi = (x * cos(ay) * cos(az) + z * sin(ay) - y * cos(ay) * sin(az)) * fx / (h * a) + Cx;
    float yi = (-z * cos(ay) * sin(ax) + x * (cos(az) * sin(ax) * sin(ay) - cos(ax) * sin(az)) + y * (-cos(ax) * cos(az) - sin(ax) * sin(ay) * sin(az))) * fx / (h * a) + Cy;
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
    float landa = sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float altitude = (-z * cos(ax) * cos(ay) + x * (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) + y * (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az))) / landa;
    float a = fx * cos(ay) * cos(az) / (landa * altitude);
    float b = -fx * cos(ay) * sin(az) / (landa * altitude);
    float c = fx * sin(ay) / (landa * altitude);
    float d = fy * (cos(az) * sin(ax) * sin(ay) - cos(ax) * sin(az)) / (landa * altitude);
    float e = fy * (-cos(ax) * cos(az) - sin(ax) * sin(ay) * sin(az)) / (landa * altitude);
    float f = -fy * (cos(ay) * sin(ax)) / (landa * altitude);
    float g = (cos(ax) * cos(az) * sin(ay) + sin(ax) * sin(az)) / landa;
    float h = (cos(az) * sin(ax) - cos(ax) * sin(ay) * sin(az)) / landa;
    float i = -(cos(ax) * cos(ay)) / landa;

    mJh << a, b, c, 0, 0, 0,
        d, e, f, 0, 0, 0,
        g, h, i, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
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
  Observation mLastObservation;
  std::chrono::steady_clock::time_point t0;
};
#include "StateFilter.inl"
#endif // SLAMMARKI_FILTER_H_