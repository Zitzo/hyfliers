#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <rgbd_tools/segmentation/color_clustering/ColorClustering.h>
#include <rgbd_tools/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <rgbd_tools/segmentation/color_clustering/types/ccsCreation.h>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <math.h>
#include "StateFilter.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pipe_detection");
    ros::NodeHandle n("~");
    StateFilter sf(n);
    ros::spin();
    return 0;
}