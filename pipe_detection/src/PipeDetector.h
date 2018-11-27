//
//
//
//

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <math.h>
#include <cstddef>

#include <rgbd_tools/segmentation/color_clustering/ColorClustering.h>
#include <rgbd_tools/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <rgbd_tools/segmentation/color_clustering/types/ccsCreation.h>
#include <rgbd_tools/object_detection/dnn/WrapperDarknet_cl.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry> 

class PipeDetector{
    public:
        PipeDetector(std::string _cfgModel, std::string weights);


    private:

        void completeContours(cv::Mat &_image);

    private:

        void depthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg);
        void colorImageCallback(const sensor_msgs::ImageConstPtr &msg);
        void IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& _imu);

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber img_sub_;
        image_transport::Publisher img_pub_;
        image_transport::Publisher img_pub2_;
        ros::Publisher pipe_pub_;
        ros::Publisher ekf_pub_;
        ros::Subscriber alt_sub_;
        ros::Publisher detector_pub;
        image_transport::Subscriber img_depth_sub_;

        cv::Mat mIntrinsic;

        cv::Mat current_depth_msg;
        sensor_msgs::Image image_depth_msg;

        rgbd::WrapperDarknet_cl detector;
}