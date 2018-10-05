#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include "ParticleFilter.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Particle_Filter");
    ros::NodeHandle n("~");
    ParticleFilter<ParticleDrone> pf(n,100);
    ros::spin();
    return 0;
}