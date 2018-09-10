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
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}
void Callback(const geometry_msgs::Twist &msg)
{
	linx = msg.linear.x;
	liny = msg.linear.y;
	linz = msg.linear.z;
	angX = msg.angular.z;
}
int main(int _argc, char **_argv)
{
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	ros::init(_argc, _argv, "publish_velocity");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/ardrone/bottom/image_raw", 1, imageCallback); //subcriptor
	ros::Subscriber sub1 = nh.subscribe("/pipe_pose", 1000, Callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher posepub = nh.advertise<geometry_msgs::PoseStamped>("/aruco/pos", 1000);
	ros::Publisher refpub = nh.advertise<geometry_msgs::PoseStamped>("/reference", 1000);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	float i;
	std::cout << "enter z altitude: ";
	std::cin >> i;
	//cv::Mat markerImage; //crear otra iamgen aruco
	//cv::aruco::drawMarker(dictionary, 24, 200, markerImage, 1);
	//cv::imwrite("marker.jpg", markerImage);

	// Open calibration file

	//VideoCapture camera(1);
	//if (!camera.isOpened()) {

	//  return -1;
	//}
	cv::Mat frame;
	Mat cameraMatrix = (Mat1d(3, 3) << 726.429011, 0.000000, 283.809411, 0.000000, 721.683494, 209.109682, 0.000000, 0.000000, 1.000000);
	Mat distCoeffs = (Mat1d(1, 5) << -0.178842, 0.660284, -0.005134, -0.005166, 0.000000);

	PID px(0.8, 0.0, 0.002, -5, 5, -20, 20);
	PID py(0.8, 0.0, 0.002, -5, 5, -20, 20);
	PID pz(0.8, 0.0, 0.002, -5, 5, -20, 20);
	PID gx(0.8, 0.0, 0.002, -5, 5, -20, 20);
	px.reference(0);
	py.reference(0);
	gx.reference(0);

	if (i != 0)
		pz.reference(i);

	auto t0 = chrono::steady_clock::now();
	while (ros::ok())
	{
		// Capture new data
		// Mat inputImage;
		// if (cv_ptr == nullptr)
		// 	continue;

		// inputImage = cv_ptr->image;
		auto t1 = chrono::steady_clock::now();
		auto rosTime = ros::Time::now();
		// if (inputImage.rows == 0)
		// 	continue;

		//cv::imshow("raw", inputImage);
		//cv::waitKey(3);
		// Detect markers
		//aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

		//aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

		// Estimate position markers
		//vector<Vec3d> vecs_orientation, vec_translations;
		//if (markerCorners.size() != 0)
		//{
		//aruco::estimatePoseSingleMarkers(markerCorners, 0.207, cameraMatrix, distCoeffs, vecs_orientation, vec_translations);
		//for (unsigned i = 0; i < vecs_orientation.size(); i++)
		//{
		//cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, vecs_orientation[i], vec_translations[i], 0.207);
		//}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
		std::cout << incT << std::endl;
		t0 = t1;
		float ux = px.update(linx, incT);
		float uy = py.update(liny, incT);
		float uz = pz.update(linz, incT);
		float ax = gx.update(angX, incT);

		//cv::imshow("image", inputImage);
		//cv::waitKey(3);

		geometry_msgs::Twist msg;
		msg.linear.x = -uy;
		msg.linear.y = ux;
		msg.linear.z = uz;
		msg.angular.z = ax;

		geometry_msgs::PoseStamped msgref;
		msgref.header.stamp = rosTime;
		msgref.pose.position.x = px.reference();
		msgref.pose.position.z = pz.reference();
		msgref.pose.position.y = py.reference();
		msgref.pose.orientation.x = gx.reference();

		geometry_msgs::PoseStamped msgpos;
		msgpos.header.stamp = rosTime;
		msgpos.pose.position.x = -linx;
		msgpos.pose.position.z = linz;
		msgpos.pose.position.y = liny;
		msgpos.pose.orientation.x = angX;

		pub.publish(msg);
		posepub.publish(msgpos);
		refpub.publish(msgref);

		// ROS_INFO_STREAM("Sending random velocity command:"
		// 				<< " linear=" << msg.linear.x << " angular=" << msg.angular.z);
		// }

		//if (markerCorners.size() == 0)
		//{

		//	float ux = 0;
		//	float uy = 0;
		//	float uz = 0;
		//}

		//cv::imshow("detections", inputImage);
		//cv::waitKey(3);
		//inputImage.rows = 0;
	}
	// return (0);
}
