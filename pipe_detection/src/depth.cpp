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
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include "ardrone_autonomy/Navdata.h"
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <geometry_msgs/PoseStamped.h>
#include <cstddef>
//#include <tf/transform_broadcaster.h>
//#include <uav_abstraction_layer/ual.h>


using namespace std;
using namespace cv;
unsigned t0, t1, t2, t3;
Point pipe_center;
Eigen::Quaternionf q;
cv::Mat current_depth_msg;
sensor_msgs::Image image_depth_msg;

// Function declarations for PCA
void drawAxis(Mat &, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, vector<Point> &, vector<Point> &, Mat &);
//void ual_init(int, char**);

// Functions

void drawAxis(Mat &img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
  double angle;
  double hypotenuse;
  angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
  hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
  //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  line(img, p, q, colour, 1, CV_AA);
  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
}
double getOrientation(const vector<Point> &pts, vector<double> &pipeCentroid, vector<double> &pipeP1, Mat &img)
{
  //Construct a buffer used by the pca analysis
  int sz = static_cast<int>(pts.size());
  Mat data_pts = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_pts.rows; ++i)
  {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }
  //Perform PCA analysis
  PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
  //Store the center of the object
  Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                     static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  pipe_center.x = cntr.x;
  pipe_center.y = cntr.y;
  //Store the eigenvalues and eigenvectors
  vector<Point2d> eigen_vecs(2);
  vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i)
  {
    eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                            pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }
  // Draw the principal components
  circle(img, cntr, 3, Scalar(255, 0, 255), 2);
  Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
  drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
  //angle = angle * 180 / M_PI;                             // Added
  //std::cout << "Centroid coordinates x,y: " << cntr.x << "," << cntr.y << std::endl;
  //std::cout << "P1 x,y: " << p1.x << "," << p1.y << std::endl;
  //std::cout << "P2 x,y: " << p2.x << "," << p2.y << std::endl;
  //std::cout << "Angle: " << angle * 180 / M_PI << std::endl;
  //std::cout << "Angle: " << angle << std::endl;
  // Add cntr point and two eigen_vecs and eigen_val (p1 and p2)
  pipeCentroid.push_back(cntr.x);
  pipeCentroid.push_back(cntr.y);
  pipeP1.push_back(p1.x);
  pipeP1.push_back(p1.y);
  if (angle > 0) 
  angle = angle*(-1);  // to avoid noise changing reference
  return angle;
}

////////////////////////////
cv::Mat src_gray;
cv::Mat dst, detected_edges;
int edgeThresh = 1;
int lowThreshold = 30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
std::string window_name = "Edge Map";
float altitude,roll,pitch;

 
void IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& _imu)
{
  //altitude = _imu->pose.position.z;
  //altitude = 1000;  // for testing
  q.x() = _imu->pose.orientation.x;
  q.y() = _imu->pose.orientation.y;
  q.z() = _imu->pose.orientation.z;
  q.w() = _imu->pose.orientation.w;
  //roll = _imu.rotX;
  //pitch = _imu.rotY;
}

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Subscriber img_depth_sub_;
  image_transport::Publisher img_pub_;
  ros::Publisher pipe_pub_;
  ros::Publisher ekf_pub_;
  ros::Subscriber alt_sub_;
  // tf::TransformBroadcaster tf_br_;
public:
  ImageProcessor(ros::NodeHandle &n) : nh_(n),
                                       it_(nh_)
  {
    //img_sub_ = it_.subscribe("/aeroarms_1/camera_0/image_raw", 1, &ImageProcessor::image_callback, this);
    img_sub_ = it_.subscribe("/camera/image", 1, &ImageProcessor::image_callback, this);
    img_depth_sub_ = it_.subscribe("/camera/image_depth", 1, &ImageProcessor::image_depth_callback, this);
   // sub_alt_ = n.subscribe("/ardrone/navdata", 1000, altitude_Callback);
    img_pub_ = it_.advertise("/output_image", 1);
    pipe_pub_ = n.advertise<geometry_msgs::PoseStamped>("/pipe_pose", 1000);
    ekf_pub_ = n.advertise<geometry_msgs::PoseStamped>("/ekf/pipe_pose", 1);
    //alt_sub_ = n.subscribe("/uav_1/mavros/local_position/pose", 1000, IMUCallback);
    alt_sub_ = n.subscribe("/mavros/local_position/pose", 1000, IMUCallback);

    //pipe_pub_ = n.advertise<geometry_msgs::Twist>("/pipe_pose", 1000);

    // Camera intrinsics
    mIntrinsic << 674.3157444517138, 0.0, 400.5, // Parrot intrinsic parameters
        0.0, 674.3157444517138, 300.5,
        0.0, 0.0, 1.0;
  }

  ~ImageProcessor() {}

 // void altitude_Callback(const sensor_msgs::ImageConstPtr &msg)
 // {
 //   cv_bridge::CvImagePtr cv_ptr;
 //   try
  //  {
  //    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //  }
  //  catch (cv_bridge::Exception &e)
  //  {
  //    ROS_ERROR("cv_bridge exception: %s", e.what());
  //    return;
  //  }
 // }

 void image_depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
 {
   cv_bridge::CvImagePtr cv_ptr_depth;
    image_depth_msg = *depth_msg;  
    try
    {
      cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    current_depth_msg = cv_ptr_depth->image; 
 }

  void image_callback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src = cv_ptr->image;
    //cv::Mat img = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);
    // cv::waitkey(30);
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    // Do something with img and store the result in send
    //ROS_INFO("Callback");
    t0 = clock();
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());

    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    /// Reduce noise with a kernel 3x3
    blur(src_gray, detected_edges, cv::Size(5, 5));

    // /// Canny detector
    // Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    // dst = Scalar::all(0);
    cv::cvtColor(src, dst, CV_BGR2HSV);
    // src.copyTo( dst, detected_edges);
    std::vector<rgbd::ImageObject> objects;
    // BOViL::ColorClusterSpace *ccs = BOViL::CreateHSVCS_8c(255,255,255);

    //grey pipe detection
    rgbd::ColorClusterSpace *ccs = rgbd::createSingleClusteredSpace(
        0, 180,
        //0, 150,
        //0, 150,
        60, 70,
        130, 150,
        180, 255, 255,
        32);



    // Red cardboard detection
   // rgbd::ColorClusterSpace *ccs = rgbd::createSingleSparseCluster(
   //     {std::pair<unsigned char, unsigned char>(0,30),std::pair<unsigned char, unsigned char>(140,180)},
   //     {std::pair<unsigned char, unsigned char>(50, 255)},
   //     {std::pair<unsigned char, unsigned char>(50, 255)},
   //     180, 255, 255,
   //     32);

    rgbd::ColorClustering<uchar>(dst.data,
                                 dst.cols,
                                 dst.rows,
                                 5000,  // minimun number of pixels detected
                                 objects,
                                 *ccs);

    cv::cvtColor(dst, dst, CV_HSV2BGR);
    cv::Mat display = src.clone();
    for (auto &ob : objects)
    {
      cv::Rect bb(
          ob.centroid().x - ob.width() / 2,
          ob.centroid().y - ob.height() / 2,
          ob.width(),
          ob.height());
      cv::rectangle(display, bb, cv::Scalar(0, 255, 0), 2);
    }
    imshow(window_name, dst);
    imshow(window_name + "_res", display);
    cv::waitKey(3);

    //Publish image
    cv_bridge::CvImage send(cv_ptr->header, cv_ptr->encoding, dst);
    img_pub_.publish(send.toImageMsg());
    t1 = clock();
    /////////// PCA
    t2 = clock();
    Mat gray;
    cvtColor(dst, gray, COLOR_BGR2GRAY);
    // Convert image to binary
    Mat bw;
    threshold(gray, bw, 50, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // Get the average of the pipes height
    float sumatory = 0;
    int num_pixel = 0;
    for (int length = 0; length < image_depth_msg.height; length++)
    {
        for (int width = 0; width < image_depth_msg.width; width++)
        {
            if(bw.at<uint16_t>(length,width) == 255)
            {
                if (current_depth_msg.at<uint16_t>(length, width)!=0)
                {
                sumatory += current_depth_msg.at<uint16_t>(length, width);
                num_pixel++;
                }
            }
        }
    }
    //float height = 0;
    if (num_pixel > 0)
        altitude = (sumatory/num_pixel)/1000;

    std::cout << "Height: " << altitude << "\n";
    

    // Find all the contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    findContours(bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); ++i)
    {
      // Calculate the area of each contour
      double area = contourArea(contours[i]);
      // Ignore contours that are too small or too large, MODIFIED AS PIPE IS A VERY LARGE OBJECT!!!
      if (area < 1e3 || 1e8 < area)
        continue;
      // Draw each contour only for visualisation purposes
      drawContours(src, contours, static_cast<int>(i), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
      vector<double> p1;
      vector<double> centroid;
      // Find the orientation of each shape
      double yaw = getOrientation(contours[i], centroid, p1, src);
      // Getting angles for EKF
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      Eigen::Matrix3f m;
      m = Eigen::AngleAxisf(euler[0]-M_PI, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(euler[1]-M_PI,  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf((yaw+M_PI/2), Eigen::Vector3f::UnitZ()); // changing value of yaw so that the position is in 0º
      //cout << "Angles: " << euler[0]-M_PI << ", " << euler[1]-M_PI << ", " << yaw+M_PI/2 << endl;
      // transform to quaternion
      Eigen::Quaternionf quaternion(m);
      // Initializaing pose
      geometry_msgs::PoseStamped pipe_data; 
      geometry_msgs::PoseStamped ekf_pipe_data; 
      // xi=fx*x/z+cx, yi=fy*y/z+cy
      pipe_data.pose.position.x = (altitude)*(pipe_center.x - mIntrinsic(0, 2)) / mIntrinsic(0, 0); // x=z*(xi-cx)/fx
      pipe_data.pose.position.y = (altitude)*(pipe_center.y - mIntrinsic(1, 2)) / mIntrinsic(1, 1); // y=z*(yi-cy)/fy
      pipe_data.pose.position.z = altitude;
      //pipe_data.pose.orientation.x = q.x();
      //pipe_data.pose.orientation.y = q.y();
      //pipe_data.pose.orientation.z = q.z() + 3.14159265/2;
      //pipe_data.pose.orientation.w = q.w();
      pipe_data.pose.orientation.x = 0;
      pipe_data.pose.orientation.y = 0;
      pipe_data.pose.orientation.z = yaw + 3.14159265/2;
      pipe_data.pose.orientation.w = 0;

      // For Kalman filter
      ekf_pipe_data.pose.position.x = pipe_center.x;
      ekf_pipe_data.pose.position.y = pipe_center.y;
      ekf_pipe_data.pose.position.z = altitude;
      ekf_pipe_data.pose.orientation.x = quaternion.x();
      ekf_pipe_data.pose.orientation.y = quaternion.y();
      ekf_pipe_data.pose.orientation.z = quaternion.z();
      ekf_pipe_data.pose.orientation.w = quaternion.w();

      pipe_pub_.publish(pipe_data);
      ekf_pub_.publish(ekf_pipe_data);
      //float altitude = pipe_data.pose.position.z;
    }
    imshow("output1", src);
    imshow("output2", gray);
    imshow("output3", bw);
    t3 = clock();
    double time1 = (double(t1 - t0) / CLOCKS_PER_SEC);
    //cout << "Execution Time Bovil: " << time1 << endl;
    double time2 = (double(t3 - t2) / CLOCKS_PER_SEC);
    //cout << "Execution Time PCA: " << time2 << endl;
  }

public:
  Eigen::Matrix<float, 3, 3> mIntrinsic;
};

int main(int _argc, char** _argv)
{

    // INIT THREADS
    ros::init(_argc, _argv, "pipe_detection");
    //cout << "pipe_detection initialized" << endl;
    ros::NodeHandle n("~");
    ImageProcessor im(n);

    ros::spin();
     return 0;
}