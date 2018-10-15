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
float batteryPercent;
int velCount, velCount100ms;
unsigned int droneState;
std::mutex stateMutex;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	//linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
}

void VelCallback(const geometry_msgs::TwistStampedConstPtr vel)
{
	// velCount++;
	// velCount100ms++;
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
	float zChange;
	std::cout << "Enter altitude: ";
	std::cin >> zChange;
	std::cout << std::endl;
	unsigned int state;

	std::thread keyboard([&]() {
		unsigned int input;
		bool run = true;
		while (run)
		{
			auto t0 = chrono::steady_clock::now();
			std::cout << "1,2,3,4=move || 9=takeoff || 8=control || 7=fakefly || 0=land" << std::endl;
			std::cin >> input;
			stateMutex.lock();
			state = input;
			stateMutex.unlock();
			auto t1 = chrono::steady_clock::now();
			float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
			if (incT > 0.6)
			{
				std::cout << "Time: " << incT << std::endl;
				switch (input)
				{
				case 0:
					std::cout << "Land" << std::endl;
					run = false;
					break;
				case 1:
					std::cout << "Forward" << std::endl;
					break;
				case 2:
					std::cout << "Backward" << std::endl;
					break;
				case 3:
					std::cout << "Left" << std::endl;
					break;
				case 4:
					std::cout << "Right" << std::endl;
					break;
				case 5:
					std::cout << " Yaw right" << std::endl;
					break;
				case 6:
					std::cout << "Yaw left" << std::endl;
					break;

				case 7:
					std::cout << "Fakefly mode" << std::endl;
					break;
				case 8:
					std::cout << "Control mode" << std::endl;
					break;
				case 9:
					std::cout << "Takeoff" << std::endl;
					break;
				case 10:
					std::cout << std::fixed << " Battery percent: " << batteryPercent << std::endl;
					std::cout << std::fixed << " ARDrone state: " << droneState << std::endl;
					break;
				case 30:
					float i;
					std::cout << "Enter altitude: ";
					std::cin >> i;
					if (i > 0 && i < 4)
					{
						std::cout << "Changing altitude reference to " << i << std::endl;
						zChange = i;
					}
					break;
				default:
					std::cout << "Hovering" << std::endl;
				}
			}
		}
		std::cout << std::fixed << "Closing keyboard thread" << std::endl;
	});
	ros::init(_argc, _argv, "MAV_Controller");
	ros::NodeHandle nh;
	ros::NodeHandle controller_node;
	ros::Rate loop_rate(20);

	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 5, Callback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);

	std_msgs::Empty emp_msg;

	ros::AsyncSpinner spinner(4);
	spinner.start();

	// INIT UAL
	grvc::ual::UAL ual(_argc, _argv);
	while (!ual.isReady() && ros::ok()) {
		std::cout << "UAL not ready!" << std::endl;
		sleep(1);
	}

	geometry_msgs::TwistStamped constant_cmd_vel;

	PID px(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID py(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID pz(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID gz(0.3, 0.0, 0.0, -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	gz.reference(0);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	if (zChange != 0)
	{
		pz.reference(zChange);
	}
	state = -1; // Start in hovering
	auto t0 = chrono::steady_clock::now();
	double keytime = 0.5; //Fixed command time
	float v = 0.5;		  // Fixed velocity
	bool fly = false;
	bool run = true;
	while (ros::ok() && run)
	{
		if (state == 8) // Control mode
		{
			auto t1 = chrono::steady_clock::now();
			auto rosTime = ros::Time::now();

			float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
			t0 = t1;
			float ux = px.update(linx, incT);
			float uy = py.update(liny, incT);
			float uz = pz.update(linz, incT);
			float az = gz.update(angZ, incT);

			geometry_msgs::TwistStamped msg = command_vel(uy, ux, uz, 0, 0, 0);

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
		else if (state == 9) // Takeoff
		{
		
			double flight_level = 2;
			ual.takeOff(flight_level);
			stateMutex.lock();
			state = 10; //To hovering mode
			stateMutex.unlock();
			fly = true;
		}
		else if (state == 0) // Land
		{
			ual.land();
			keyboard.join();
			std::cout << "Closing mav_controller" << std::endl;
			run = false;
			exit(0);
		}
		else if (state == 1) // go forward
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(v, 0, 0, 0, 0, 0);
				vel_pub.publish(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 2) // go backward
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(-v, 0, 0, 0, 0, 0);
				vel_pub.publish(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 3) // go right
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(0, v, 0, 0, 0, 0);
				vel_pub.publish(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 4) // go left
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(0, -v, 0, 0, 0, 0);
				vel_pub.publish(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 5) // yaw left
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(0, 0, 0, 0, 0, 0.2);
				ual.setVelocity(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 6) // yaw right
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + keytime)
			{
				constant_cmd_vel = command_vel(0, 0, 0, 0, 0, -0.2);
				ual.setVelocity(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if (state == 30)
		{
			pz.reference(zChange);
		}
		else // hovering mode
		{
			constant_cmd_vel = command_vel(0, 0, 0, 0, 0, 0);
            ual.setVelocity(constant_cmd_vel);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}