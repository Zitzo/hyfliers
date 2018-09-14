/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah
This program launches the AR Drone. 
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ardrone_autonomy/Navdata.h" 
#include <std_msgs/Empty.h>

using namespace cv;
using namespace std;
std_msgs::Empty emp_msg; 

int main(int _argc, char **_argv)
{
	ROS_INFO("Flying ARdrone");
	ros::init(_argc, _argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Publisher pub_takeoff;
	pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */

 	while (ros::ok()) 
 	{
		double time_start=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start+5.0) /* Send command for five seconds*/
		{ 
			pub_takeoff.publish(emp_msg); /* launches the drone */
			ros::spinOnce();
			loop_rate.sleep();
		}//time loop
		ROS_INFO("ARdrone launched");
		exit(0);
	}//ros::ok loop

	return (0);
}
