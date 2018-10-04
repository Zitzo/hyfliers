#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "ParticleFilter.h"
#include <iostream>

class ParticleDrone : public rgbd::Particle
{
  public:
    ParticleDrone(){
        //setNoise(0.1, 0.1, 10.0);
        // set(	((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*2*M_PI);
        // 		std::cout << "inited" << std::endl;
    };
    void simulate(){
        //move(0.1, 0.5);
    };
    void calcWeigh(Particle &_realParticle){
        //mWeigh = measurementProb(static_cast<ParticleRobot &>(_realParticle).sense());
    };

    //operator std::array<double, 3>(){ return position(); }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Particle_Filter");
    ros::NodeHandle n("~");
    ParticleFilter<ParticleDrone> pf(n,100);
    ros::spin();
    return 0;
}