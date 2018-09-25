StateFilter::StateFilter(ros::NodeHandle &n) : nh_(n)
{
    pipe_subscriber = n.subscribe("/ekf/pipe_pose", 10, &StateFilter::pipeDetectionCallback, this);
    filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/ekf_pose", 1);
}

/*------------------------------------------------------------------------------------------------------------------------*/

void StateFilter::pipeDetectionCallback(const geometry_msgs::PoseStamped msg)
{
    mLastObservation.xi = msg.pose.position.x;
    mLastObservation.yi = msg.pose.position.y;
    mLastObservation.altitude = msg.pose.position.z;
    mLastObservation.time = std::chrono::steady_clock::now();
    mLastObservation.quat = Eigen::Quaternionf(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    if (!mKalmanInitialized)
    {
        float fx = 726.429011;
        float fy = 721.683494;
        float Cx = 283.809411;
        float Cy = 209.109682;
        mIntrinsic << fx, 0, Cx,
            0, fy, Cy,
            0, 0, 1;
        initializeKalmanFilter();
        t0 = std::chrono::steady_clock::now();
    }
    else
    {
        float incT = std::chrono::duration_cast<std::chrono::milliseconds>(mLastObservation.time - t0).count() / 1000.0f;
        // Adding new observation
        computeKalmanFilter(incT);
        t0 = std::chrono::steady_clock::now();
    }
}
//rostopic pub /ekf/pipe_pose geotry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, pose: {position: {x: 0, y: 0, z: 1}, orientation: {w: 1.0}}}'
/*------------------------------------------------------------------------------------------------------------------------*/

bool StateFilter::computeKalmanFilter(float _incT)
{
    Eigen::Matrix3f Rot = mLastObservation.quat.normalized().toRotationMatrix();
    double ax = atan2(Rot(2, 1), Rot(2, 2));
    double ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
    double az = atan2(Rot(1, 0), Rot(0, 0));
    Eigen::Matrix<float, 6, 1> z;
    z << mLastObservation.xi, mLastObservation.yi, mLastObservation.altitude, ax, ay, az;

    // New step in EKF
    ekf.stepEKF(z, _incT);
    auto filtered = ekf.state();
    geometry_msgs::PoseStamped filteredPose;
    filteredPose.pose.position.x = filtered(0);
    filteredPose.pose.position.y = filtered(1);
    filteredPose.pose.position.z = filtered(2);
    filteredPose.pose.orientation.x = filtered(3);
    filteredPose.pose.orientation.y = filtered(4);
    filteredPose.pose.orientation.z = filtered(5);
    filteredPose.pose.orientation.w = 0;
    std::cout << "Filtered pose: "
              << " x: " << filtered(0) << " y: " << filtered(1) << " z: " << filtered(2);
    std::cout << " ax: " << filtered(3) << " ay: " << filtered(4) << " az: " << filtered(5) << std::endl;
    filtered_pub.publish(filteredPose);
}

/*------------------------------------------------------------------------------------------------------------------------*/

void StateFilter::initializeKalmanFilter()
{
    ROS_INFO("Initializating Extended Kalman Filter");
    Eigen::Matrix<float, 6, 6> mQ; // State covariance
    mQ.setIdentity();
    mQ *= 0.01;
    Eigen::Matrix<float, 6, 6> mR; // Observation covariance
    mR.setIdentity();
    Eigen::Matrix3f Rot = mLastObservation.quat.normalized().toRotationMatrix();
    double ax = atan2(Rot(2, 1), Rot(2, 2));
    double ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
    double az = atan2(Rot(1, 0), Rot(0, 0));
    float Zc = (mLastObservation.altitude * sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1) / ((1 / cos(ax) * (1 / cos(ay)))));
    float Xc = (mLastObservation.xi * (-Zc * (1 / cos(ax) * (1 / cos(ay)) / mIntrinsic(0, 0) + mIntrinsic(0, 2)) / ((1 / cos(az) * (1 / cos(ay))))));
    float Yc = (mLastObservation.yi * (-Zc * (1 / cos(ax) * (1 / cos(ay)) / mIntrinsic(1, 1) + mIntrinsic(1, 2)) * (cos(ax) * cos(az) - sin(ax) * sin(ay) * sin(az))));

    Eigen::Matrix<float, 6, 1> x0;
    x0 << Xc, Yc, Zc, ax, ay, az;

    ekf.setUpEKF(mQ, mR, x0);
    mKalmanInitialized = true;
}