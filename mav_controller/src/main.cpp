#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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
#include "ardrone_autonomy/Navdata.h"
#include <std_msgs/Empty.h>

using namespace cv;
using namespace pcl;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
float linx = 0, liny = 0, linz = 0, angZ = 0;
float batteryPercent;
unsigned int state;
int velCount, velCount100ms;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	//linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
}

void IMUCallback(const ardrone_autonomy::Navdata imu)
{
	linz = imu.altd;
	linz = linz / 1000;
	batteryPercent = imu.batteryPercent;
	state = imu.state;
}

void VelCallback(const geometry_msgs::TwistConstPtr vel)
{
	velCount++;
	velCount100ms++;
}
//nuevo

int main(int _argc, char **_argv)
{
	float i;
	std::cout << "enter z altitude: ";
	std::cin >> i;
	std::cout << std::endl;
	bool fly = true;

	std::thread keyboard([&]() {
		int input;
		for (;;)
		{
			std::cout << "Type 66 to quit: ";
			std::cin >> input;
			std::cout << std::endl;
			if (input == 66)
			{
				std::cout << "66 pressed ending test" << std::endl;
				fly = false;
			}
			else
			{
				std::cout << "66 not pressed" << std::endl;
			}
		}
	});
	ros::init(_argc, _argv, "MAV_Controller");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 10, Callback);
	ros::Subscriber alt_sub = nh.subscribe("/ardrone/navdata", 10, IMUCallback);
	ros::Subscriber vel_sub = nh.subscribe(nh.resolveName("cmd_vel"), 50, VelCallback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);

	ros::Publisher pub_empty = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
	std_msgs::Empty emp_msg;

	ros::AsyncSpinner spinner(4);
	spinner.start();
	////nuevo
	geometry_msgs::Twist twist_msg_pshover;
	twist_msg_pshover.linear.x = 0.03; // lowest value for psudo hover to work
	twist_msg_pshover.linear.y = 0.03;
	twist_msg_pshover.linear.z = 0.0;
	twist_msg_pshover.angular.x = 0.0;
	twist_msg_pshover.angular.y = 0.0;
	twist_msg_pshover.angular.z = 0.03;

	cv::Mat frame;
	Mat cameraMatrix = (Mat1d(3, 3) << 726.429011, 0.000000, 283.809411, 0.000000, 721.683494, 209.109682, 0.000000, 0.000000, 1.000000);
	Mat distCoeffs = (Mat1d(1, 5) << -0.178842, 0.660284, -0.005134, -0.005166, 0.000000);

	PID px(0.0, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID py(0.0, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID pz(1.5, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID gz(0.0, 0.0, 0.0, -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	gz.reference(0);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	if (i != 0)
	{
		pz.reference(i);
	}

	auto t0 = chrono::steady_clock::now();
	while (ros::ok() && fly)
	{
		//nuevo
		if (0)
		{
			vel_pub.publish(twist_msg_pshover);

			ROS_INFO("Flying");
		} //fl
		else
		{
			auto t1 = chrono::steady_clock::now();
			auto rosTime = ros::Time::now();

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
			float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
			t0 = t1;
			float ux = px.update(linx, incT);
			float uy = py.update(liny, incT);
			float uz = pz.update(linz, incT);
			float az = gz.update(angZ, incT);

			geometry_msgs::Twist msg;
			msg.linear.x = 0.03; //uy;
			msg.linear.y = 0.03; //ux;
			msg.linear.z = uz;
			msg.angular.z = 0.02; //az;
			// Hovering deactivated
			msg.angular.x = 1;
			msg.angular.y = 1;

			// if ()
			// {
			// 	std::cout << std::fixed << " Battery percent: " << batteryPercent << std::endl;
			// }
			//std::cout << std::fixed << " ARDrone state: " << state << std::endl;

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

			vel_pub.publish(msg);
			pose_pub.publish(msgpos);
			ref_pub.publish(msgref);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	while (ros::ok())
	{
		ROS_INFO("ARdrone Landing");
		double time_start = (double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec() < time_start + 2.0)
		{
			pub_empty.publish(emp_msg); //launches the drone
			ros::spinOnce();
			loop_rate.sleep();
		}
		ROS_INFO("ARdrone landed");
		exit(0);
	} //ros::ok
	  // return (0);
}
