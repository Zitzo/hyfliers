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
        mIntrinsic << 674.3157444517138, 0, 400.5,
            0, 674.3157444517138, 300.5,
            0, 0, 1;
    };

    void simulate()
    {

        Eigen::Vector3f rot;
        rot << atan2(mOrientation(2, 1), mOrientation(2, 2)),
            atan2(-mOrientation(2, 0), sqrt(mOrientation(2, 1) * mOrientation(2, 1) + mOrientation(2, 2) * mOrientation(2, 2))),
            atan2(mOrientation(1, 0), mOrientation(0, 0));

        // Create translation noise
        Eigen::Vector3f transNoise;
        transNoise << gauss(0, 0.05), gauss(0, 0.05), gauss(0, 0.01);
        // Create rotation noise
        Eigen::Vector3f rotNoise;
        rotNoise << gauss(0, M_PI / 4), gauss(0, M_PI / 4), gauss(0, M_PI / 4);

        // Move particle
        mPosition += transNoise;
        rot += rotNoise;

        mOrientation = Eigen::AngleAxisf(rot(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rot(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rot(0), Eigen::Vector3f::UnitX());
    };

    double computeWeight(Observation &_Particle)
    {
        stateToObservation();

        // Distance weight between 1 and 0
        double weightDistance =1- sqrt((_Particle.xi-mObservation.xi)*(_Particle.xi-mObservation.xi)+(_Particle.yi-mObservation.yi)*(_Particle.yi-mObservation.yi))/(sqrt(640*640+480*480));
        
        Eigen::Matrix3f Rot = _Particle.quat.normalized().toRotationMatrix();
        double drone_ax = atan2(Rot(2, 1), Rot(2, 2));
        double drone_ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
        double drone_az = atan2(Rot(1, 0), Rot(0, 0));
        double particle_ax = atan2(mOrientation(2, 1), mOrientation(2, 2));
        double particle_ay = atan2(-mOrientation(2, 0), sqrt(mOrientation(2, 1) * mOrientation(2, 1) + mOrientation(2, 2) * mOrientation(2, 2)));
        double particle_az = atan2(mOrientation(1, 0), mOrientation(0, 0));

        double ax_diff = (drone_ax- particle_ax)*(drone_ax- particle_ax);
        double ay_diff = (drone_ay- particle_ay)*(drone_ay- particle_ay);
        double az_diff = (drone_az- particle_az)*(drone_az- particle_az);

        //  Orientation weight between 1 and 0
        double weightOrientation =1- sqrt(ax_diff+ay_diff+az_diff)/(2*M_PI);

        return weightDistance*weightOrientation;
    };

    // Init particles near a pose
    void initParticle(Eigen::Matrix4f initPose_)
    {

        Eigen::Vector3f trans = initPose_.block<3, 1>(2, 2);
        Eigen::Matrix3f rot = initPose_.block<3, 3>(0, 0);

        Eigen::Vector3f ea = rot.eulerAngles(0, 1, 2);

        // Create translation noise
        Eigen::Vector3f transNoise;
        transNoise << gauss(0, 1), gauss(0, 1), gauss(0, 0.5);

        // Create rotation noise
        Eigen::Vector3f rotNoise;
        rotNoise << gauss(0, M_PI / 2), gauss(0, M_PI / 2), gauss(0, M_PI / 2);

        mPosition = trans + transNoise;
        ea += rotNoise;

        mOrientation = Eigen::AngleAxisf(ea(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(ea(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(ea(0), Eigen::Vector3f::UnitX());
    }

    double gauss(const double &_nu, const double &_sigma)
    {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::normal_distribution<> d(_nu, _sigma);

        return d(gen);
    }

  public:
    /// Obtain world position from camera observation
    void observationToState(Observation obs)
    {
        // Obtain roll, pitch and yaw
        Eigen::Matrix3f Rot = obs.quat.normalized().toRotationMatrix();
        double ax = atan2(Rot(2, 1), Rot(2, 2));
        double ay = atan2(-Rot(2, 0), sqrt(Rot(2, 1) * Rot(2, 1) + Rot(2, 2) * Rot(2, 2)));
        double az = atan2(Rot(1, 0), Rot(0, 0));

        // Obtain camera position from observation
        float Zc = obs.altitude * sqrt(tan(ax) * tan(ax) + tan(ay) * tan(ay) + 1);
        float Xc = ((obs.xi - mIntrinsic(0, 2)) / mIntrinsic(0, 0)) * Zc;
        float Yc = ((obs.yi - mIntrinsic(1, 2)) / mIntrinsic(1, 1)) * Zc;
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
    };

  private:
    /// Obtain observation from world position
    void stateToObservation()
    {
        // Get orientation
        Eigen::Vector3f ea = mOrientation.eulerAngles(0, 1, 2);
        float ax = ea(0);
        float ay = ea(1);
        float az = ea(2);

        // Transformation camera-drone

        Eigen::Matrix4f Trot;
        Trot.setIdentity();
        Eigen::Matrix3f R; // Change: Rotation camera-IMU
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, -1;
        Trot.block<3, 3>(0, 0) = R;
        Eigen::Vector3f T;
        T << 0, 0, 0; // Change:  Translation camera-IMU
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
        mObservation = z;
    };

    // Get rotation matrix from roll, pitch and yaw
    Eigen::Matrix4f create_rotation_matrix(const float ax_, const float ay_, const float az_, bool inverse)
    {
        Eigen::Matrix3f rx, ry, rz;
        rx = Eigen::AngleAxisf(ax_, Eigen::Vector3f::UnitX());
        ry = Eigen::AngleAxisf(ay_, Eigen::Vector3f::UnitY());
        rz = Eigen::AngleAxisf(az_, Eigen::Vector3f::UnitZ());

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

  public:
    Eigen::Matrix4f getPose()
    {
        Eigen::Matrix4f pose;
        pose.block<3, 3>(0, 0) = mOrientation;
        pose.block<3, 1>(2, 2) = mPosition;
        return pose;
    }

  public:
    Observation mObservation;

  private:
    Eigen::Vector3f mPosition;
    Eigen::Matrix3f mOrientation;
    Eigen::Matrix<float, 3, 3> mIntrinsic;
};

template <typename ParticleType_, typename ObservationData_>
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
    rgbd::ParticleFilterCPU<ParticleType_, ObservationData_> filter;

    Observation mLastObservation;
    std::chrono::steady_clock::time_point t0;
    bool mParticleInitialized = false;
    ParticleDrone drone;
};

#include "ParticleFilter.inl"
#endif // SLAMMARKI_PARTICLE_FILTER_H_