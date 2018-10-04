template<typename ParticleType_>
ParticleFilter<ParticleType_>::ParticleFilter(ros::NodeHandle &n, unsigned _numParticles) : nh_(n), filter(_numParticles)
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
template<typename ParticleType_>
void ParticleFilter<ParticleType_>::pipeDetectionCallback(const geometry_msgs::PoseStamped msg)
{

    if (!mParticleInitialized)
    {
        initializeParticleFilter();
    }
    else
    {
        //filter.step(robot);
    }
}

/*------------------------------------------------------------------------------------------------------------------------*/
template<typename ParticleType_>
bool ParticleFilter<ParticleType_>::computeParticleFilter()
{
}

/*------------------------------------------------------------------------------------------------------------------------*/
template<typename ParticleType_>
void ParticleFilter<ParticleType_>::initializeParticleFilter()
{
    // filter(1000);
    // filter.init();
}