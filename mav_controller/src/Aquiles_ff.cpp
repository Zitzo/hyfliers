#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include "std_msgs/Float64.h"
#include <thread>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>

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

    grvc::ual::Waypoint waypoint = ual.pose();
	waypoint.pose.position.y +=2;

    ual.goToWaypoint(waypoint);
    std::cout << "Arrived!" << std::endl;

    // Set linear velocity 2 seconds

    geometry_msgs::TwistStamped msg;
	msg.twist.linear.x = 1;
	msg.twist.linear.y = 0;
	msg.twist.linear.z = 0;
	msg.twist.angular.z = 0;
	msg.twist.angular.x = 0;
	msg.twist.angular.y = 0;

    ual.setVelocity(msg);

    sleep(2);

    // Land the UAV
    ual.land();
}
