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
    cv::Mat display(640, 480, CV_8UC3, cv::Scalar(0,0,0));
    filter.step(mLastObservation);
    
    // Draw drone
    cv::circle(display, cv::Point2i(mLastObservation.xi, mLastObservation.yi), 3, cv::Scalar(255,0,0), 3);
    // Draw particles
    std::vector<ParticleDrone> particles = filter.particles();
    for(auto& particle: particles){
        cv::circle(display, cv::Point2i(particle.mObservation.xi, particle.mObservation.yi), 0.5, cv::Scalar(0,0,255), 3);
    }    

}

/*------------------------------------------------------------------------------------------------------------------------*/
template <typename ParticleType_, typename ObservationData_>
void ParticleFilter<ParticleType_, ObservationData_>::initializeParticleFilter()
{
    ROS_INFO("Initializating particle filter");
    drone.observationToState(mLastObservation);
    filter.init();

    // Initialize particle poses.
    Eigen::Matrix4f dronePose = drone.getPose();
    std::vector<ParticleDrone> particles = filter.particles();

    for(auto& particle: particles){
        particle.initParticle(dronePose);
    }

    cv::namedWindow("display", CV_WINDOW_FREERATIO);
}