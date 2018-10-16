#include <chrono>
#include <thread>
#include <stdlib.h>
#include <mutex>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <camera_info_manager/camera_info_manager.h>
#include "PID.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/Empty.h>
#include "std_msgs/Float64.h"
#include <uav_abstraction_layer/ual.h>

using namespace cv;
using namespace pcl;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
float linx = 0, liny = 0, linz = 0, angZ = 0;
std::mutex stateMutex;
unsigned state = 1;
ros::Time lastPoseTime;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	//linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
	lastPoseTime = ros::Time::now();
}

geometry_msgs::TwistStamped command_vel(float _ux, float _uy, float _uz, float _ax, float _ay, float _az)
{
	geometry_msgs::TwistStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.twist.linear.x = _ux;
	msg.twist.linear.y = _uy;
	msg.twist.linear.z = _uz;
	msg.twist.angular.x = _ax;
	msg.twist.angular.y = _ay;
	msg.twist.angular.z = _az;
	return msg;
}

int main(int _argc, char **_argv)
{
	ros::init(_argc, _argv, "MAV_Controller");
	ros::NodeHandle nh;
	ros::NodeHandle controller_node;
	ros::Rate loop_rate(30);

	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 5, Callback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);

	std_msgs::Empty emp_msg;
	geometry_msgs::TwistStamped constant_cmd_vel;

	ros::AsyncSpinner spinner(4);
	spinner.start();

	// INIT UAL
	grvc::ual::UAL ual(_argc, _argv);
	while (!ual.isReady() && ros::ok())
	{
		std::cout << "UAL not ready!" << std::endl;
		sleep(1);
	}

	std::thread keyboard([&]() {
		unsigned int input;
		bool run = true;
		while (run)
		{
			auto t0 = chrono::steady_clock::now();
			std::cout << "1: TakeOff || 2: GoToWaypoint || 3: Control || 4: Land" << std::endl;
			std::cin >> input;
			auto t1 = chrono::steady_clock::now();
			float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
			if (incT > 0.6)
			{
				std::cout << "Time: " << incT << std::endl;
				switch (input)
				{
				case 1:
					std::cout << "Takeoff" << std::endl;
					break;
				case 2:
					std::cout << "GoToWaypoint" << std::endl;
					break;
				case 3:
					std::cout << "Control" << std::endl;
					break;
				case 4:
					std::cout << "Land" << std::endl;
					break;
				default:
					std::cout << "No command selected" << std::endl;
				}
			}
			// Changing state
			stateMutex.lock();
			state = input;
			stateMutex.unlock();
		}
		std::cout << std::fixed << "Closing keyboard thread" << std::endl;
	});

	PID px(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID py(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID pz(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID gz(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	pz.reference(3); // 3 meters altitude control
	gz.reference(0);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	auto t0 = chrono::steady_clock::now();
	float v = 0.5; // Fixed velocity
	grvc::ual::Waypoint home;
	home.header.frame_id = "map";
	home.pose.position.x = 0;
	home.pose.position.y = 0;
	home.pose.position.z = 2;
	home.pose.orientation.x = 0;
	home.pose.orientation.y = 0;
	home.pose.orientation.z = 0;
	home.pose.orientation.w = 1;

	grvc::ual::Waypoint waypoint;
	waypoint.header.frame_id = "map";
	waypoint.pose.position.x = 2;
	waypoint.pose.position.y = 0;
	waypoint.pose.position.z = 3;
	waypoint.pose.orientation.x = 0;
	waypoint.pose.orientation.y = 0;
	waypoint.pose.orientation.z = 0;
	waypoint.pose.orientation.w = 1;
	bool run = true;
	while (ros::ok() && run)
	{
		if (state == 1)
		{ // Takeoff
			std::cout << "Initiating TakeOff" << std::endl;
			double flight_level = 2;
			ual.takeOff(flight_level);
			std::cout << "TakeOff completed" << std::endl;
			stateMutex.lock();
			state = 2;
			stateMutex.unlock();
			std::cout << "Changed state to GoToWaypoint mode" << std::endl;
		}
		else if (state == 2) // goToWaypoint
		{
			std::cout << "Going to waypoint at " << waypoint.pose.position.x << "," << waypoint.pose.position.y << "," << waypoint.pose.position.z << "," << std::endl;
			ual.goToWaypoint(waypoint);
			std::cout << "Arrived!" << std::endl;
			stateMutex.lock();
			state = 3;
			stateMutex.unlock();
			std::cout << "Changed state to control mode" << std::endl;
		}
		else if (state == 3) // Control mode
		{	
			auto rosTime = ros::Time::now();
			if (abs(lastPoseTime.toSec() - rosTime.toSec()) > 0.5)
			{
				std::cout << "Not detected pipe in more than 0.5s. Changed state to GoToWayPoint" << std::endl;
				stateMutex.lock();
				state = 2;
				stateMutex.unlock();
			}
			else
			{
				auto t1 = chrono::steady_clock::now();
				float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
				t0 = t1;
				float ux = px.update(linx, incT);
				float uy = py.update(liny, incT);
				float uz = pz.update(linz, incT);
				float az = gz.update(angZ, incT);

				geometry_msgs::TwistStamped msg = command_vel(uy, ux, uz, 0, 0, 0); // Change to control yaw too

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

				ual.setVelocity(msg);
			}
		}
		else if (state == 4) // Land
		{
			std::cout << "Going home" << std::endl;
			ual.goToWaypoint(home);
			std::cout << "Arrived home" << std::endl;
			ual.land();
			std::cout << "Landed" << std::endl;
			std::cout << "Closing mav_controller" << std::endl;
			run = false;
			exit(0);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}