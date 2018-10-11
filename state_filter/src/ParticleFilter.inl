template <typename ParticleType_, typename ObservationData_>
ParticleFilter<ParticleType_, ObservationData_>::ParticleFilter(ros::NodeHandle &n, unsigned _numParticles) : nh_(n), filter(_numParticles)
{
    pipe_subscriber = n.subscribe("/ekf/pipe_pose", 10, &ParticleFilter::pipeDetectionCallback, this);
    filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/particle_pose", 1);
    no_Filtered_pub = n.advertise<geometry_msgs::PoseStamped>("/ekf_pose_nFilter", 1);
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
    filter.step(mLastObservation);
}

/*------------------------------------------------------------------------------------------------------------------------*/
template <typename ParticleType_, typename ObservationData_>
void ParticleFilter<ParticleType_, ObservationData_>::initializeParticleFilter()
{
    ROS_INFO("Initializating particle filter");
    drone.observationToState(mLastObservation);
    filter.init();

}