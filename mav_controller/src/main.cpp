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
#include <mutex>

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

void IMUCallback(const ardrone_autonomy::Navdata imu)
{
	linz = imu.altd;
	linz = linz / 1000;
	batteryPercent = imu.batteryPercent;
	droneState = imu.state;
}

void VelCallback(const geometry_msgs::TwistConstPtr vel)
{
	// velCount++;
	// velCount100ms++;
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
			std::cout << "1,2,3,4=move || 9=takeoff || 8=control || 7=fakefly || 0=land || 4=Battery and State || 30=change Z reference" << std::endl;
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
				case 6:
					std::cout << std::fixed << " Battery percent: " << batteryPercent << std::endl;
					std::cout << std::fixed << " ARDrone state: " << droneState << std::endl;
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
				case 30:
					float i;
					std::cout << "Enter altitude: ";
					std::cin >> i;
					if (i>0 && i<4){
						std::cout << "Changing altitude reference to " << i << std::endl;
						zChange=i;
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
	ros::Rate loop_rate(20);

	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 5, Callback);
	ros::Subscriber alt_sub = nh.subscribe("/ardrone/navdata", 5, IMUCallback);
	ros::Subscriber vel_sub = nh.subscribe(nh.resolveName("cmd_vel"), 5, VelCallback);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"), 1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/pos", 5);
	ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/mav_controller/reference", 5);

	ros::Publisher pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher pub_empty = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
	std_msgs::Empty emp_msg;

	ros::AsyncSpinner spinner(4);
	spinner.start();

	geometry_msgs::Twist constant_cmd_vel;

	PID px(1.0, 0.01, 0.3, -0.5, 0.5, -20, 20);
	PID py(1.0, 0.01, 0.3, -0.5, 0.5, -20, 20);
	PID pz(1.5, 0.0, 0.0, -0.5, 0.5, -20, 20);
	PID gz(1.0, 0.01, 0.3, -0.5, 0.5, -20, 20);

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
	state = 10; // Start in hovering
	auto t0 = chrono::steady_clock::now();
	bool run = true;
	double keytime = 0.5; //Fixed command time
	float v = 0.5;		  // Fixed velocity
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

			geometry_msgs::Twist msg;
			msg.linear.x = uy; //uy;
			msg.linear.y = ux; //ux;
			msg.linear.z = uz;
			msg.angular.z = az;	//rad/s
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
			msgpos.pose.position.x = -liny;
			msgpos.pose.position.z = linz;
			msgpos.pose.position.y = linx;
			msgpos.pose.orientation.z = angZ;

			vel_pub.publish(msg);
			pose_pub.publish(msgpos);
			ref_pub.publish(msgref);
		}
		else if (state == 9) // Takeoff
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + 2.0) /* Send command for five seconds*/
			{
				pub_takeoff.publish(emp_msg); /* launches the drone */
				ros::spinOnce();
				loop_rate.sleep();
			} //time loop
			stateMutex.lock();
			state = 10; //To hovering mode
			stateMutex.unlock();
		}
		else if (state == 7) // fakefly mode
		{
			constant_cmd_vel.linear.x = 0.03;
			constant_cmd_vel.linear.y = 0.03;
			constant_cmd_vel.linear.z = 0.0;
			constant_cmd_vel.angular.x = 0.0;
			constant_cmd_vel.angular.y = 0.0;
			constant_cmd_vel.angular.z = 0.03;
			vel_pub.publish(constant_cmd_vel);
		}
		else if (state == 0) // Land
		{
			double time_start = (double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec() < time_start + 2.0)
			{
				pub_empty.publish(emp_msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
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
				constant_cmd_vel.linear.x = v;
				constant_cmd_vel.linear.y = 0.0;
				constant_cmd_vel.linear.z = 0.0;
				constant_cmd_vel.angular.x = 0.0;
				constant_cmd_vel.angular.y = 0.0;
				constant_cmd_vel.angular.z = 0.0;
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
				constant_cmd_vel.linear.x = -v;
				constant_cmd_vel.linear.y = 0.0;
				constant_cmd_vel.linear.z = 0.0;
				constant_cmd_vel.angular.x = 0.0;
				constant_cmd_vel.angular.y = 0.0;
				constant_cmd_vel.angular.z = 0.0;
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
				constant_cmd_vel.linear.x = 0.0;
				constant_cmd_vel.linear.y = v;
				constant_cmd_vel.linear.z = 0.0;
				constant_cmd_vel.angular.x = 0.0;
				constant_cmd_vel.angular.y = 0.0;
				constant_cmd_vel.angular.z = 0.0;
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
				constant_cmd_vel.linear.x = 0.0;
				constant_cmd_vel.linear.y = -v;
				constant_cmd_vel.linear.z = 0.0;
				constant_cmd_vel.angular.x = 0.0;
				constant_cmd_vel.angular.y = 0.0;
				constant_cmd_vel.angular.z = 0.0;
				vel_pub.publish(constant_cmd_vel);
				ros::spinOnce();
				loop_rate.sleep();
			}
			stateMutex.lock();
			state = 10; // go hovering mode
			stateMutex.unlock();
		}
		else if(state == 30)
		{
			pz.reference(zChange);
		}
		else // hovering mode
		{
			constant_cmd_vel.linear.x = 0.0;
			constant_cmd_vel.linear.y = 0.0;
			constant_cmd_vel.linear.z = 0.0;
			constant_cmd_vel.angular.x = 0.0;
			constant_cmd_vel.angular.y = 0.0;
			constant_cmd_vel.angular.z = 0.0;
			vel_pub.publish(constant_cmd_vel);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
