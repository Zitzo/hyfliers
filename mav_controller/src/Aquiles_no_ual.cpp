	#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include "std_msgs/Float64.h"
#include <thread>
#include <chrono>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "MAV_Controller");
	ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);

    ros::AsyncSpinner spinner(4);
	spinner.start();
    int state = 0;
    while(state!=1){
        std::cout << "Ready to go: press 1 to start!" << std::endl;
        std::cin >> state;
    }
	
    geometry_msgs::TwistStamped msg;
	msg.twist.linear.x = 0;
	msg.twist.linear.y = 1;
	msg.twist.linear.z = 0;
	msg.twist.angular.z = 0;
	msg.twist.angular.x = 0;
	msg.twist.angular.y = 0;
    
	for(;;){
		vel_pub.publish(msg);
		loop_rate.sleep();
		//std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
    sleep(2);
}
