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

using namespace cv;
using namespace pcl;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
float linx, liny, linz, angX, angY, angZ;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
	
}
int main(int _argc, char **_argv)
{

	ros::init(_argc, _argv, "publish_velocity");
	ros::NodeHandle nh;

	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 1000, Callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher posepub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 1000);
	ros::Publisher refpub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 1000);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	float i;
	std::cout << "enter z altitude: ";
	std::cin >> i;

	cv::Mat frame;
	Mat cameraMatrix = (Mat1d(3, 3) << 726.429011, 0.000000, 283.809411, 0.000000, 721.683494, 209.109682, 0.000000, 0.000000, 1.000000);
	Mat distCoeffs = (Mat1d(1, 5) << -0.178842, 0.660284, -0.005134, -0.005166, 0.000000);

	PID px(0.06, 0.0, 0.0, -5, 5, -20, 20);
	PID py(0.06, 0.0, 0.0, -5, 5, -20, 20);
	PID pz(0.05, 0.0, 0.0, -5, 5, -20, 20);
	PID gz(0.05, 0.0, 0.0, -5, 5, -20, 20);

	px.reference(0);
	py.reference(0);
	gz.reference(0);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	if (i != 0)
		pz.reference(i);

	auto t0 = chrono::steady_clock::now();
	while (ros::ok())
	{

		auto t1 = chrono::steady_clock::now();
		auto rosTime = ros::Time::now();

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
		std::cout << incT << std::endl;
		t0 = t1;
		float ux = px.update(linx, incT);
		float uy = py.update(liny, incT);
		float uz = pz.update(linz, incT);
		float az = gz.update(angZ, incT);

		geometry_msgs::Twist msg;
		msg.linear.x = uy;
		msg.linear.y = ux;
		msg.linear.z = uz;
		msg.angular.z = az;
		// Hovering deactivated
		msg.angular.x = 1;
		msg.angular.y = 1;

		geometry_msgs::PoseStamped msgref;
		msgref.header.stamp = rosTime;
		msgref.pose.position.x = px.reference();
		msgref.pose.position.z = pz.reference();
		msgref.pose.position.y = py.reference();
		msgref.pose.orientation.x = gz.reference();

		geometry_msgs::PoseStamped msgpos;
		msgpos.header.stamp = rosTime;
		msgpos.pose.position.x = liny;
		msgpos.pose.position.z = linz;
		msgpos.pose.position.y = linx;
		msgpos.pose.orientation.z = angZ;

		pub.publish(msg);
		posepub.publish(msgpos);
		refpub.publish(msgref);
	}
	// return (0);
}
