#include "ardrone_autonomy/Navdata.h"

StateFilter::StateFilter(ros::NodeHandle &n) : nh_(n)
{
    centroid_subscriber = n.subscribe("/ardrone/navdata", 10, &StateFilter::centroidCallback, this);
    if (newData)
    {

        if (!mKalmanInitialized)
        {
            t0 = std::chrono::steady_clock::now();
            initializeKalmanFilter();
        }
        else
        {
            auto t1 = std::chrono::steady_clock::now();
            float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
            t0 = t1;
            computeKalmanFilter(incT);
        }
        newData = false;
    }
}

/*------------------------------------------------------------------------------------------------------------------------*/

void StateFilter::centroidCallback(const ardrone_autonomy::Navdata imu)
{
    newData = true;
}

/*------------------------------------------------------------------------------------------------------------------------*/

bool StateFilter::computeKalmanFilter(float _incT)
{

    // Adding new observation
    Eigen::Matrix<float, 6, 1> z;
    // We assume z cte between centroid and dominant axis
    // z << centroid[0], centroid[1], p1,
    //     p1[0], p1[1], p1;

    // New step in EKF
    ekf.stepEKF(z, _incT);
    Eigen::Matrix<float, 4, 1> XfilteredCntr = ekf.state();

    // State model to observation model to draw it
    Eigen::Matrix<float, 6, 1> ZfilteredCntr;
    ZfilteredCntr.setIdentity();
    // ZfilteredCntr << mIntrinsic(0, 0) * XfilteredCntr[0] / XfilteredCntr[2] + mIntrinsic(0, 2), mIntrinsic(1, 1) * XfilteredCntr[1] / XfilteredCntr[2] + mIntrinsic(1, 2), 1,
    //     mIntrinsic(0, 0) * XfilteredCntr[3] / XfilteredCntr[5] + mIntrinsic(0, 2), mIntrinsic(1, 1) * XfilteredCntr[4] / XfilteredCntr[5] + mIntrinsic(1, 2), 1;

    // Filtered centroid
    cv::Point filtCentroid = {(int)ZfilteredCntr[0], (int)ZfilteredCntr[1]};
    cv::Point filtP1 = {(int)ZfilteredCntr[3], (int)ZfilteredCntr[4]};
    cv::Point P1C = filtP1 - filtCentroid;
    double filteredAngle = atan2(P1C.y, P1C.x);
    std::cout << "Filtered centroid coordinates x,y: " << filtCentroid.x << "," << filtCentroid.y << std::endl;
    std::cout << "Filtered P1 coordinates x,y: " << filtP1.x << "," << filtP1.y << std::endl;
    std::cout << "Filtered angle: " << filteredAngle << std::endl;
}

/*------------------------------------------------------------------------------------------------------------------------*/

void StateFilter::initializeKalmanFilter()
{
    ROS_INFO("Initializating Extended Kalman Filter");
    Eigen::Matrix<float, 4, 4> mQ; // State covariance
    mQ.setIdentity();
    mQ *= 0.01;
    Eigen::Matrix<float, 6, 6> mR; // Observation covariance
    mR.setIdentity();
    Eigen::Matrix<float, 4, 1> x0;
    // x0 << (centroid[0] - mIntrinsic(0, 2)) / mIntrinsic(0, 0), (centroid[1] - mIntrinsic(1, 2)) / mIntrinsic(1, 1), p1,
    //     (p1[0] - mIntrinsic(0, 2)) / mIntrinsic(0, 0), (p1[1] - mIntrinsic(1, 2)) / mIntrinsic(1, 1), p1;
    ekf.setUpEKF(mQ, mR, x0);
    mKalmanInitialized = true;
}