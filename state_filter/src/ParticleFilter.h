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

struct Observation
{
    float altitude;
    float xi;
    float yi;
    Eigen::Quaternionf quat;
    std::chrono::steady_clock::time_point time;
};

class ParticleDrone : public rgbd::ParticleInterface<Observation>
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
    double computeWeight(Observation &_Particle){
        //mWeigh = measurementProb(static_cast<ParticleRobot &>(_realParticle).sense());
    };

  private:
    /// Obtain world position from camera observation
    Eigen::Vector3f observationToState(float xi_, float yi_, float h_, Eigen::Quaternionf q_)
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
        Eigen::Matrix4f Tworld_drone;
        Tworld_drone = create_rotation_matrix(ax, ay, az, false);

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
        Eigen::Matrix4f Tworld_camera = Tworld_drone * Tdrone_camera;
        Eigen::Vector4f world_position = Tworld_camera * cameraPosition;
        mOrientation = Tworld_camera.block<3, 3>(0, 0);
        mPosition = world_position.block<3, 1>(0, 0);
        return world_position.block<3, 1>(0, 0);
    };

    /// Obtain observation from world position
    Observation stateToObservation()
    {
        // Get orientation
        Eigen::Vector3f ea = mOrientation.eulerAngles(0, 1, 2);
        float ax = ea(0);
        float ay = ea(1);
        float az = ea(2);

        // Transformation camera-drone
        Eigen::Matrix3f R;
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, -1;
        Eigen::Matrix4f Trot;
        Trot.setIdentity();
        Trot.block<3, 3>(0, 0) = R;
        Eigen::Vector3f T;
        T << 0, 0, 0; // Translation camera-IMU
        Eigen::Matrix4f Ttrans;
        Ttrans.setIdentity();
        Ttrans.block<3, 1>(2, 2) = T;

        Eigen::Matrix4f Tcamera_drone = Trot.transpose() * (-Ttrans);

        //Transformation drone-world
        //Eigen::Matrix4f rot = create_rotation_matrix(ax, ay, az, true);
        Eigen::Matrix4f Tdrone_world;
        Tdrone_world = create_rotation_matrix(ax, ay, az, true);

        Eigen::Vector4f worldPosition;
        worldPosition << mPosition(0), mPosition(1), mPosition(2), 1;

        // Camera position
        auto cameraPosition = Tcamera_drone * Tdrone_world * worldPosition;

        float Xc = cameraPosition(0);
        float Yc = cameraPosition(1);
        float Zc = cameraPosition(2);

        // Observation
        Observation z;
        z.xi = mIntrinsic(0, 0) * Xc / Zc + mIntrinsic(0, 2);
        z.yi = mIntrinsic(1, 1) * Xc / Zc + mIntrinsic(1, 2);
        z.altitude = Zc / (sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1));
        return z;
    };

    // Get rotation matrix from roll, pitch and yaw
    Eigen::Matrix4f create_rotation_matrix(const float ax_,const float ay_,const float az_, bool inverse)
    {
        Eigen::Matrix3f rx,ry,rz;
        rx = Eigen::AngleAxisf(ax_ * M_PI, Eigen::Vector3f::UnitX());
        ry = Eigen::AngleAxisf(ay_ * M_PI, Eigen::Vector3f::UnitY());
        rz = Eigen::AngleAxisf(az_ * M_PI, Eigen::Vector3f::UnitZ());

        Eigen::Matrix4f rotMatrix;
        rotMatrix.setIdentity();
        if (inverse)
        {
            Eigen::Matrix3f rotationMatrix = rz.transpose() * ry.transpose() * rx.transpose();
            rotMatrix.block<3, 3>(0, 0) = rotationMatrix;
        }
        else
        {
            Eigen::Matrix3f rotationMatrix = rz * ry * rx;
            rotMatrix.block<3, 3>(0, 0) = rotationMatrix;
        }
        return rotMatrix;
    }

  private:
    Eigen::Vector3f mPosition;
    Eigen::Matrix3f mOrientation;
    Eigen::Matrix<float, 3, 3> mIntrinsic;
};

template <typename ParticleType_,typename ObservationData_>
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
    rgbd::ParticleFilterCPU<ParticleType_,ObservationData_> filter;
    Eigen::Matrix<float, 3, 3> mIntrinsic;
    bool mParticleInitialized = false;
    Observation mLastObservation;
    std::chrono::steady_clock::time_point t0;
};

#include "ParticleFilter.inl"
#endif // SLAMMARKI_PARTICLE_FILTER_H_