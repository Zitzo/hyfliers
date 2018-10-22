#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <stdlib.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <camera_info_manager/camera_info_manager.h>
#include "PID.h"
#include <chrono>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <thread>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <uav_abstraction_layer/ual.h>
#include "std_msgs/Float64.h"

#include <thread>

using namespace cv;
using namespace pcl;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
float linx=0, liny=0, linz=0, angZ=0;
int velCount,velCount100ms;
float reference_z = 2;


void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	//linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
	
}

void Callback_ekf(const geometry_msgs::PoseStamped &msg)
{
	//linx = msg.pose.position.x;
	//liny = -1*msg.pose.position.y;
	//linz = -1*msg.pose.position.z;
	//angZ = msg.pose.orientation.z;
	
}

void IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& imu) 
{ 
  linz = imu->pose.position.z; 
} 

void VelCallback(const geometry_msgs::TwistConstPtr vel)
{
    velCount++;
    velCount100ms++;
}

int main(int _argc, char **_argv)
{

    // INIT UAL
	grvc::ual::UAL ual(_argc, _argv);
	while (!ual.isReady() && ros::ok()) {
		std::cout << "UAL not ready!" << std::endl;
		sleep(1);
	}

    // Takeoff and go to waypoint 

    double flight_level = 2;

	ual.takeOff(flight_level);

    grvc::ual::Waypoint waypoint;
	waypoint.header.frame_id = "map";
	waypoint.pose.position.x = 0;
	waypoint.pose.position.y = 0;
	waypoint.pose.position.z = 4;
	waypoint.pose.orientation.x = 0;
	waypoint.pose.orientation.y = 0;
	waypoint.pose.orientation.z = 0;
	waypoint.pose.orientation.w = 1;

    ual.goToWaypoint(waypoint);
    std::cout << "Arrived!" << std::endl;


    // UAV in position

    // Topics Init
	ros::init(_argc, _argv, "publish_velocity");
	ros::NodeHandle nh;

	ros::Subscriber sub1 = nh.subscribe("/ekf_pose", 10, Callback_ekf);
	ros::Subscriber sub2 = nh.subscribe("/pipe_pose", 10, Callback);
	ros::Subscriber alt_sub = nh.subscribe("/uav_1/mavros/local_position/pose", 10, IMUCallback); 
	//ros::Subscriber alt_sub = nh.subscribe("/mavros/local_position/pose", 10, IMUCallback);
	ros::Subscriber vel_sub	= nh.subscribe(nh.resolveName("cmd_vel"),50, VelCallback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/simulated/cmd_vel", 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	cv::Mat frame;

	PID px(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID py(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID pz(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID gz(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	gz.reference(0);
	pz.reference(reference_z);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	std::thread zRefThread(
		[&](){
			for(;;){
				std::cout << "Set Z reference: ";
			    std::cin >> reference_z;

				if (reference_z == 0)
				ual.land();
				if (reference_z == -1)
				{
				ual.takeOff(flight_level);
				}
				if (reference_z > 0)
				pz.reference(reference_z);
			}
		}
	);

	auto t0 = chrono::steady_clock::now();
	while (ros::ok())
	{

		auto t1 = chrono::steady_clock::now();
		auto rosTime = ros::Time::now();

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
		//std::cout << incT << std::endl;
		t0 = t1;
		if (reference_z > 0)
		{
		float ux = px.update(linx, incT);
		float uy = py.update(liny, incT);
		float uz = pz.update(linz, incT);
		float az = gz.update(angZ, incT);
		
		geometry_msgs::TwistStamped msg;
		msg.twist.linear.x = uy;
		msg.twist.linear.y = ux;
		msg.twist.linear.z = uz;
		msg.twist.angular.z = az;
		msg.twist.angular.x = 0;
		msg.twist.angular.y = 0;

		geometry_msgs::PoseStamped msgref;
		msgref.header.stamp = rosTime;
		msgref.pose.position.x = px.reference();
		msgref.pose.position.z = pz.reference();
		msgref.pose.position.y = py.reference();
		msgref.pose.orientation.x = gz.reference();

		geometry_msgs::PoseStamped msgpos;
		msgpos.header.stamp = rosTime;
		msgpos.pose.position.x = -liny;
		msgpos.pose.position.z = linz;
		msgpos.pose.position.y = linx;
		msgpos.pose.orientation.z = angZ;

       ual.setVelocity(msg);

		vel_pub.publish(msg);
		pose_pub.publish(msgpos);
		ref_pub.publish(msgref);
		}
	}
	// return (0);
}
