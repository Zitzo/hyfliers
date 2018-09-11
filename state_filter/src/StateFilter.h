#ifndef SLAMMARKI_FILTER_H_
#define SLAMMARKI_FILTER_H_

#include <ros/ros.h>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
// #include "geometry_msgs/Twist.h"

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
    mHZk << (fx * mXfk[0] / mXfk[2]) + Cx, (fy * mXfk[1] / mXfk[2]) + Cy, 1,
        (fx * mXfk[3] / mXfk[5]) + Cx, (fy * mXfk[4] / mXfk[5]) + Cy, 1;
  }
  void updateJh()
  {
    float fx = 726.429011;
    float fy = 721.683494;
    mJh << fx / mXfk[0], 0, -fx * mXfk[0] / (mXfk[2] * mXfk[2]), 0, 0, 0,
        0, fy / mXfk[1], -fy * mXfk[1] / (mXfk[2] * mXfk[2]), 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, fx / mXfk[3], 0, -fx * mXfk[3] / (mXfk[5] * mXfk[5]),
        0, 0, 0, 0, fy / mXfk[4], -fy * mXfk[4] / (mXfk[5] * mXfk[5]),
        0, 0, 0, 0, 0, 1;
  }
};

void centroidCallback(const ardrone_autonomy::Navdata imu){};
void p1Callback(const ardrone_autonomy::Navdata imu){};
void imuCallback(const ardrone_autonomy::Navdata imu){};
class StateFilter
{
public:
  StateFilter(ros::NodeHandle &n) : nh_(n)
  {
    centroid_subscriber = n.subscribe("/ardrone/navdata", 1000, centroidCallback);
    p1_subscriber = n.subscribe("/ardrone/navdata", 1000, p1Callback);
    imu_subscriber = n.subscribe("/ardrone/navdata", 1000, imuCallback);
    if (!mKalmanInitialized)
    {
      initializeKalmanFilter();
    }
    else
    {
      computeKalmanFilter();
    }
  };
  
  bool computeKalmanFilter()
  {
    std::cout << "Initializating Extended Kalman Filter" << std::endl;
    // Adding new observation
    Eigen::Matrix<float, 6, 1> z;
    // We assume z cte between centroid and dominant axis
    z << centroid[0], centroid[1], p1,
        p1[0], p1[1], p1;
    double rate = 0.2; //666: Rate!!!!!!!!!

    // New step in EKF
    ekf.stepEKF(z, rate);
    Eigen::Matrix<float, 6, 1> XfilteredCntr = ekf.state();

    // State model to observation model to draw it
    Eigen::Matrix<float, 6, 1> ZfilteredCntr;
    ZfilteredCntr.setIdentity();
    ZfilteredCntr << mIntrinsic(0, 0) * XfilteredCntr[0] / XfilteredCntr[2] + mIntrinsic(0, 2), mIntrinsic(1, 1) * XfilteredCntr[1] / XfilteredCntr[2] + mIntrinsic(1, 2), 1,
        mIntrinsic(0, 0) * XfilteredCntr[3] / XfilteredCntr[5] + mIntrinsic(0, 2), mIntrinsic(1, 1) * XfilteredCntr[4] / XfilteredCntr[5] + mIntrinsic(1, 2), 1;

    // Filtered centroid
    cv::Point filtCentroid = {(int)ZfilteredCntr[0], (int)ZfilteredCntr[1]};
    cv::Point filtP1 = {(int)ZfilteredCntr[3], (int)ZfilteredCntr[4]};
    cv::Point P1C = filtP1 - filtCentroid;
    double filteredAngle = atan2(P1C.y, P1C.x);
    std::cout << "Filtered centroid coordinates x,y: " << filtCentroid.x << "," << filtCentroid.y << std::endl;
    std::cout << "Filtered P1 coordinates x,y: " << filtP1.x << "," << filtP1.y << std::endl;
    std::cout << "Filtered angle: " << filteredAngle << std::endl;
  }

  // Initialize EKF with first observation of centroid, p1 and altitude
  void initializeKalmanFilter()
  {
    Eigen::Matrix<float, 6, 6> mQ; // State covariance
    mQ.setIdentity();
    mQ *= 0.01;
    Eigen::Matrix<float, 6, 6> mR; // Observation covariance
    mR.setIdentity();
    Eigen::Matrix<float, 6, 1> x0;
    x0 << (centroid[0] - mIntrinsic(0, 2)) / mIntrinsic(0, 0), (centroid[1] - mIntrinsic(1, 2)) / mIntrinsic(1, 1), p1,
        (p1[0] - mIntrinsic(0, 2)) / mIntrinsic(0, 0), (p1[1] - mIntrinsic(1, 2)) / mIntrinsic(1, 1), p1;
    ekf.setUpEKF(mQ, mR, x0);
    mKalmanInitialized = true;
  }

public:
  ros::NodeHandle nh_;
  ros::Publisher pipe_pub_;
  ros::Subscriber centroid_subscriber;
  ros::Subscriber p1_subscriber;
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