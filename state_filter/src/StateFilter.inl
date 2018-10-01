template <typename Type_, int D1_, int D2_>
StateFilter<Type_, D1_, D2_>::StateFilter(ros::NodeHandle &n) : nh_(n)
{
    pipe_subscriber = n.subscribe("/ekf/pipe_pose", 10, &StateFilter::pipeDetectionCallback, this);
    filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/ekf_pose", 1);
    no_Filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/ekf_pose_nFilter", 1);
    float fx = 674.3157444517138;
    float fy = 674.3157444517138;
    float Cx = 400.5;
    float Cy = 300.5;
    mIntrinsic << fx, 0, Cx,
        0, fy, Cy,
        0, 0, 1;
}

/*------------------------------------------------------------------------------------------------------------------------*/

template <typename Type_, int D1_, int D2_>
void StateFilter<Type_, D1_, D2_>::pipeDetectionCallback(const geometry_msgs::PoseStamped msg)
{
    mLastObservation.xi = msg.pose.position.x;
    mLastObservation.yi = msg.pose.position.y;
    mLastObservation.altitude = msg.pose.position.z;
    mLastObservation.time = std::chrono::steady_clock::now();
    mLastObservation.quat = Eigen::Quaternionf(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    if (!mKalmanInitialized)
    {
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

/*------------------------------------------------------------------------------------------------------------------------*/

template <typename Type_, int D1_, int D2_>
bool StateFilter<Type_, D1_, D2_>::computeKalmanFilter(float _incT)
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
    filteredPose.header.stamp = ros::Time::now();
    filteredPose.pose.position.x = filtered(0);
    filteredPose.pose.position.y = filtered(1);
    filteredPose.pose.position.z = filtered(2);
    filteredPose.pose.orientation.x = filtered(3);
    filteredPose.pose.orientation.y = filtered(4);
    filteredPose.pose.orientation.z = filtered(5);
    filteredPose.pose.orientation.w = 0;
    filtered_pub.publish(filteredPose);

    Rot = mLastObservation.quat.normalized().toRotationMatrix();
    ax = atan2(Rot(2, 1), Rot(2, 2));
    ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
    az = atan2(Rot(1, 0), Rot(0, 0));
    float Zc = (mLastObservation.altitude / sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1));
    float Xc = (mLastObservation.xi - mIntrinsic(0, 2)) * Zc / mIntrinsic(0, 0);
    float Yc = (mLastObservation.yi - mIntrinsic(1, 2)) * Zc / mIntrinsic(1, 1);
    geometry_msgs::PoseStamped noFilteredPose;
    noFilteredPose.header.stamp = ros::Time::now();
    noFilteredPose.pose.position.x = Xc;
    noFilteredPose.pose.position.y = Yc;
    noFilteredPose.pose.position.z = Zc;
    noFilteredPose.pose.orientation.x = ax;
    noFilteredPose.pose.orientation.y = ay;
    noFilteredPose.pose.orientation.z = az;
    noFilteredPose.pose.orientation.w = 0;
    no_Filtered_pub.publish(noFilteredPose);
}

/*------------------------------------------------------------------------------------------------------------------------*/
template <typename Type_, int D1_, int D2_>
void StateFilter<Type_, D1_, D2_>::initializeKalmanFilter()
{
    ROS_INFO("Initializating Extended Kalman Filter");
    Eigen::Matrix<float, 6, 6> mQ; // State covariance
    mQ.setIdentity();
    mQ *= 0.1;
    Eigen::Matrix<float, 6, 6> mR; // Observation covariance
    mR.setIdentity();
    mR *= 0.1;
    Eigen::Matrix3f Rot = mLastObservation.quat.normalized().toRotationMatrix();
    double ax = atan2(Rot(2, 1), Rot(2, 2));
    double ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
    double az = atan2(Rot(1, 0), Rot(0, 0));
    float Zc = mLastObservation.altitude / sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
    float Xc = (mLastObservation.xi - mIntrinsic(0, 2)) * Zc / mIntrinsic(0, 0);
    float Yc = (mLastObservation.yi - mIntrinsic(1, 2)) * Zc / mIntrinsic(1, 1);
    float Xm = Xc*cos(ay)*cos(az) + Yc*(sin(ax)*sin(ay)*cos(az) + sin(az)*cos(ax)) + Zc*(sin(ax)*sin(az) - sin(ay)*cos(ax)*cos(az));
    float Ym = -Xc*sin(az)*cos(ay) + Yc*(-sin(ax)*sin(ay)*sin(az) + cos(ax)*cos(az)) + Zc*(sin(ax)*cos(az) + sin(ay)*sin(az)*cos(ax));
    float Zm = -Xc*sin(ay) + Yc*sin(ax)*cos(ay) - Zc*cos(ax)*cos(ay);

    Eigen::Matrix<float, 6, 1> x0;
    x0 << Xm, Ym, Zm, ax, ay, az;

    ekf.setUpEKF(mQ, mR, x0);
    mKalmanInitialized = true;
}