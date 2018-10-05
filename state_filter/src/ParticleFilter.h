#ifndef SLAMMARKI_PARTICLE_FILTER_H_
#define SLAMMARKI_PARTICLE_FILTER_H_

#include <ros/ros.h>
#include <rgbd_tools/state_filtering/ParticleFilterCPU.h>
#include <Eigen/Eigen>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64.h"
#include <ctime>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <math.h>

class ParticleDrone : public rgbd::Particle
{
  public:
    ParticleDrone(){
        //setNoise(0.1, 0.1, 10.0);
        // set(	((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*2*M_PI);
    };
    void simulate(){
        //move(0.1, 0.5);
    };
    void calcWeigh(Particle &_realParticle){
        //mWeigh = measurementProb(static_cast<ParticleRobot &>(_realParticle).sense());
    };

    //operator std::array<double, 3>(){ return position(); }
};

struct Observation
{
    float altitude;
    float xi;
    float yi;
    Eigen::Quaternionf quat;
    std::chrono::steady_clock::time_point time;
};

template<typename ParticleType_>
class ParticleFilter
{
  public:
    ParticleFilter(ros::NodeHandle &n, unsigned _numParticles);

    void pipeDetectionCallback(const geometry_msgs::PoseStamped msg);

  private:
    void initializeParticleFilter();

    bool computeParticleFilter();

  public:
    ros::NodeHandle nh_;
    ros::Publisher filtered_pub;
    ros::Publisher no_Filtered_pub;
    ros::Subscriber pipe_subscriber;
    rgbd::ParticleFilterCPU<ParticleType_> filter;
    Eigen::Matrix<float, 3, 3> mIntrinsic;
    bool mParticleInitialized = false;
    Observation mLastObservation;
    std::chrono::steady_clock::time_point t0;
};

#include "ParticleFilter.inl"
#endif // SLAMMARKI_PARTICLE_FILTER_H_