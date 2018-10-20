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
float linx_ekf = 0, liny_ekf = 0, linz_ekf = 0, angZ_ekf = 0;
std::mutex stateMutex;
std::mutex cinMutex;
unsigned int state = -1;
// unsigned int previous_state = -1;  // if necesary for more security
ros::Time lastPoseTime;
float reference_z = 3;
int security = -1;
int kalman = 0;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	//linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
	lastPoseTime = ros::Time::now();
}

void Callback_ekf(const geometry_msgs::PoseStamped &msg)
{
	linx_ekf = msg.pose.position.x;
	liny_ekf = -1*msg.pose.position.y;
	//linz = -1*msg.pose.position.z;
	angZ_ekf = msg.pose.orientation.z;
	
}

void IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& imu) 
{ 
  linz = imu->pose.position.z; // Comment if use of depth image
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
	std::thread keyboard([&]() {
		std::cout << "Initializating keyboard thread" << std::endl;
		unsigned int input;
		bool run = true;
		while (run)
		{
			auto t0 = chrono::steady_clock::now();
			std::cout << "0: Repose mode ||1: TakeOff || 2: GoToWaypoint || 3: Control || 4: Reference change of z || 5: Land on pipe || 6: Land at home || 7: Mav_controller close || 8: Waypoint change || 9:Kalman change. " << std::endl;
			sleep(1);
			cinMutex.lock();
			std::cin >> input;
			cinMutex.unlock();
			auto t1 = chrono::steady_clock::now();
			float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
			if (incT > 0.6)
			{
				std::cout << "Time: " << incT << std::endl;
				switch (input)
				{
				case 0:
					std::cout << "Repose mode" << std::endl;
					break;
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
					std::cout << "Reference change of z" << std::endl;
					break;
				case 5:
					std::cout << "Land on pipe" << std::endl;
					break;
				case 6:
					std::cout << "Land at home" << std::endl;
					break;
				case 7:
					std::cout << "Mav_controller close" << std::endl;
					break;
				case 8:
					std::cout << "Waypoint change" << std::endl;
					break;
				case 9:
					std::cout << "Kalman change" << std::endl;
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

	ros::init(_argc, _argv, "MAV_Controller");
	ros::NodeHandle nh;
	ros::NodeHandle controller_node;
	ros::Rate loop_rate(30);

	ros::Subscriber sub1 = nh.subscribe("/ekf_pose", 10, Callback_ekf);
	ros::Subscriber sub2 = nh.subscribe("/pipe_pose", 5, Callback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);
	//ros::Subscriber alt_sub = nh.subscribe("/mavros/local_position/pose", 10, IMUCallback);  //real
	ros::Subscriber alt_sub = nh.subscribe("/uav_1/mavros/local_position/pose", 10, IMUCallback); // simulation

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

	PID px(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID py(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID pz(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);
	PID gz(0.2, 0.00, 0.0, -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	pz.reference(reference_z); // altitude control
	gz.reference(0);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	auto t0 = chrono::steady_clock::now();
	grvc::ual::Waypoint home;   // It should go there if it loose the pipe
	home.header.frame_id = "map";
	home.pose.position.x = -5;
	home.pose.position.y = -5;
	home.pose.position.z = 3;
	home.pose.orientation.x = 0;
	home.pose.orientation.y = 0;
	home.pose.orientation.z = 0;
	home.pose.orientation.w = 1;

	grvc::ual::Waypoint waypoint;  // Position on the pipe 
	waypoint.header.frame_id = "map";
	waypoint.pose.position.x = 0;
	waypoint.pose.position.y = 0;
	waypoint.pose.position.z = 4;
	waypoint.pose.orientation.x = 0;
	waypoint.pose.orientation.y = 0;
	waypoint.pose.orientation.z = 0;
	waypoint.pose.orientation.w = 1;
	bool run = true;

	std::cout << "No mode selected, select mode 0-8" << std::endl;
	while (ros::ok() && run)
	{
		if (state == 0) // Repose mode
		{ 
			if (security == -1)
			{
				stateMutex.lock();
				state = -1;
				stateMutex.unlock();
				std::cout << "This mode can't be run before first takeoff" << std::endl;
			}
			if (security == 0)
			{
				std::cout << "Not secure to change to repose mode, changing to control mode" << std::endl;
				stateMutex.lock();
				state = 3;
				stateMutex.unlock();
				std::cout << "Changed state to control mode" << std::endl;
			}
		}
		else if (state == 1) // Takeoff mode
		{ 
			security= 0;
			std::cout << "Initiating TakeOff" << std::endl;
			double flight_level = 2;
			ual.takeOff(flight_level);
			std::cout << "TakeOff completed" << std::endl;
			stateMutex.lock();
			state = 2;
			stateMutex.unlock();
			std::cout << "Changed state to GoToWaypoint mode" << std::endl;
		}
		else if (state == 2) // GoToWaypoint mode
		{
			security= 0;
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
			security= 0;
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
				float ux, uy, uz, az;
				if (kalman == 0)
				{
					 ux = px.update(linx, incT);
					 uy = py.update(liny, incT);
					 uz = pz.update(linz, incT);
					 az = gz.update(angZ, incT);
				}
				else
				{
					 ux = px.update(linx_ekf, incT);
					 uy = py.update(liny_ekf, incT);
					 uz = pz.update(linz, incT);
					 az = gz.update(angZ_ekf, incT);	
				}

				geometry_msgs::TwistStamped msg = command_vel(uy, ux, uz, 0, 0, az); 

				geometry_msgs::PoseStamped msgref;
				msgref.header.stamp = rosTime;
				msgref.pose.position.x = px.reference();
				msgref.pose.position.z = pz.reference();
				msgref.pose.position.y = py.reference();
				msgref.pose.orientation.x = gz.reference();

				geometry_msgs::PoseStamped msgpos;

				if (kalman == 0)
				{
					msgpos.header.stamp = rosTime;
					msgpos.pose.position.x = -liny;
					msgpos.pose.position.z = linz;
					msgpos.pose.position.y = linx;
					msgpos.pose.orientation.z = angZ;
				}
				else 
				{
					msgpos.header.stamp = rosTime;
					msgpos.pose.position.x = -liny_ekf;
					msgpos.pose.position.z = linz;
					msgpos.pose.position.y = linx_ekf;
					msgpos.pose.orientation.z = angZ_ekf;
				}
				vel_pub.publish(msg);
				pose_pub.publish(msgpos);
				ref_pub.publish(msgref);

				ual.setVelocity(msg);
			}
		}
		else if (state == 4) // Reference change
		{
				cinMutex.lock();
				std::cout << "Set Z reference: ";
			    std::cin >> reference_z;

				if (reference_z > 0)
				pz.reference(reference_z);

				std::cout << "Altitude reference changed" << std::endl;
				std::cout << "Changing to control mode" << std::endl;
				stateMutex.lock();
				state = 3;
				stateMutex.unlock();
				std::cout << "Changed state to control mode" << std::endl;
				cinMutex.unlock();
		}
		else if (state == 5) // Land on pipe
		{
			std::cout << "Landing on pipe" << std::endl;
			ual.land();
			std::cout << "Landed" << std::endl;
			security = 1;
			std::cout << "Changing to repose mode" << std::endl;
			stateMutex.lock();
			state = 0;
			stateMutex.unlock();
			std::cout << "Changed to repose mode" << std::endl;
		}
		else if (state == 6) // secure Land mode on home 
		{
			std::cout << "Going home" << std::endl;
			ual.goToWaypoint(home);
			std::cout << "Arrived home" << std::endl;
			ual.land();
			std::cout << "Landed" << std::endl;
			security = 1;
			std::cout << "Changing to repose mode" << std::endl;
			stateMutex.lock();
			state = 0;
			stateMutex.unlock();
			std::cout << "Changed to repose mode" << std::endl;
		}
		else if (state == 7) // Mav_controller close 
		{			
			if (security == 0)
			{
				std::cout << "Not secure to close mav_controller mode, changing to control mode" << std::endl;
				stateMutex.lock();
				state = 3;
				stateMutex.unlock();
				std::cout << "Changed state to control mode" << std::endl;
			}
			else
			{ int close = -1;
				std::cout << "Are you sure you want to close mav_controller: yes=1 no=0 ";
				if (close == 1)
				{
				std::cin >> close;
				std::cout << "Closing mav_controller" << std::endl;
				run = false;
				exit(0);
				}
			}
		}
		else if (state == 8)  // Waypoint change
		{		
				cinMutex.lock();
				float wp_value= 0;
				std::cout << "Changing waypoint value";
				std::cout << "X value of waypoint: ";
			    std::cin >> wp_value;
				waypoint.pose.position.x = wp_value;
				std::cout << "Y value of waypoint: ";
			    std::cin >> wp_value;
				waypoint.pose.position.y = wp_value;
				std::cout << "Z value of waypoint: ";
			    std::cin >> wp_value;
				waypoint.pose.position.z = wp_value;
				std::cout << "Changing to control mode" << std::endl;
				stateMutex.lock();
				state = 3;
				stateMutex.unlock();
				std::cout << "Changed state to control mode" << std::endl;
				cinMutex.unlock();
		}
		else if (state == 9) // Takeoff mode
		{ 
			if (kalman == 0)
			{
				kalman = 1;
				std::cout << "Changed to Kalman mode" << std::endl;
			}
			else
			{
				kalman = 0;
				std::cout << "Changed to NO Kalman mode" << std::endl;
			}
			std::cout << "Changing to control mode" << std::endl;
			stateMutex.lock();
			state = 3;
			stateMutex.unlock();
			std::cout << "Changed state to control mode" << std::endl;
		}
		else{
			// std::cout << "No mode selected" << std::endl;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}