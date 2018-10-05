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
    ParticleDrone()
    {
        //setNoise(0.1, 0.1, 10.0);
        // set(	((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*WORLD_SIZE,
        // 		((double)rand()/RAND_MAX)*2*M_PI);
        mIntrinsic << 674.3157444517138, 0, 400.5,
            0, 674.3157444517138, 300.5,
            0, 0, 1;
    };
    void simulate(){
        //move(0.1, 0.5);
    };
    void calcWeigh(Particle &_realParticle){
        //mWeigh = measurementProb(static_cast<ParticleRobot &>(_realParticle).sense());
    };

  private:
    /// Obtain world position from camera observation
    Eigen::Vector4f observationToState(float xi_, float yi_, float h_, Eigen::Quaternionf q_)
    {
        // Obtain roll, pitch and yaw
        Eigen::Matrix3f Rot = q_.normalized().toRotationMatrix();
        double ax = atan2(Rot(2, 1), Rot(2, 2));
        double ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
        double az = atan2(Rot(1, 0), Rot(0, 0));

        // Obtain camera position from observation
        float Zc = h_ * sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
        float Xc = ((xi_ - mIntrinsic(0, 2)) / mIntrinsic(0, 0)) * Zc;
        float Yc = ((yi_ - mIntrinsic(1, 2)) / mIntrinsic(1, 1)) * Zc;
        Eigen::Vector4f cameraPosition;
        cameraPosition << Xc, Yc, Zc, 1;

        // Transformation world-drone
        Eigen::Affine3f rot = create_rotation_matrix(ax, ay, az);
        Eigen::Matrix4f Tworld_drone = rot.matrix();

        // Transformation drone-camera
        Eigen::Matrix4f Tdrone_camera;
        Eigen::Matrix3f R;
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, -1;
        Eigen::Vector3f T;
        T << 0, 0, 0;
        Tdrone_camera.setIdentity();
        Tdrone_camera.block<3, 3>(0, 0) = R;
        Tdrone_camera.block<3, 1>(2, 2) = T;

        return Tworld_drone * Tdrone_camera * cameraPosition;
    };
    /// Obtain observation from world position
    Eigen::Vector4f stateToObservation(){

    };

    // Get rotation matrix from roll, pitch and yaw
    Eigen::Affine3f create_rotation_matrix(float ax, float ay, float az)
    {
        Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
        Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
        Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
        return rz * ry * rx;
    }

  private:
    Eigen::Vector4f Position;
    Eigen::Matrix<float, 3, 3> mIntrinsic;
};

struct Observation
{
    float altitude;
    float xi;
    float yi;
    Eigen::Quaternionf quat;
    std::chrono::steady_clock::time_point time;
};

template <typename ParticleType_>
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