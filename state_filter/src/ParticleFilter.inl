template <typename ParticleType_, typename ObservationData_>
ParticleFilter<ParticleType_, ObservationData_>::ParticleFilter(ros::NodeHandle &n, unsigned _numParticles) : nh_(n), filter(_numParticles)
{
    pipe_subscriber = n.subscribe("/ekf/pipe_pose", 10, &ParticleFilter::pipeDetectionCallback, this);
    filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/particle_pose", 1);
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
template <typename ParticleType_, typename ObservationData_>
void ParticleFilter<ParticleType_, ObservationData_>::pipeDetectionCallback(const geometry_msgs::PoseStamped msg)
{
    mLastObservation.xi = msg.pose.position.x;
    mLastObservation.yi = msg.pose.position.y;
    mLastObservation.altitude = msg.pose.position.z;
    mLastObservation.time = std::chrono::steady_clock::now();
    mLastObservation.quat = Eigen::Quaternionf(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

    if (!mParticleInitialized)
    {
        initializeParticleFilter();
    }
    else
    {
        computeParticleFilter();
    }
    t0 = std::chrono::steady_clock::now();
}

/*------------------------------------------------------------------------------------------------------------------------*/
template <typename ParticleType_, typename ObservationData_>
bool ParticleFilter<ParticleType_, ObservationData_>::computeParticleFilter()
{
}

/*------------------------------------------------------------------------------------------------------------------------*/
template <typename ParticleType_, typename ObservationData_>
void ParticleFilter<ParticleType_, ObservationData_>::initializeParticleFilter()
{
    filter.init();
}