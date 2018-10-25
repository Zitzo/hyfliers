#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include "PlatformDetector.h"

using namespace std;
using namespace cv;
unsigned t0, t1, t2, t3;
Point pipe_center;
Eigen::Quaternionf q;

// Function declarations for PCA
void drawAxis(Mat &, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, vector<Point> &, vector<Point> &, Mat &);
//void ual_init(int, char**);

// Functions

void drawAxis(Mat &img, Point p, Point q, Scalar colour, const float scale = 0.2) {
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

double getOrientation(const vector<Point> &pts, vector<double> &pipeCentroid, vector<double> &pipeP1, Mat &img) {
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
PlatformDetector gPiperDetector;
 
void IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& _imu) {
  altitude = _imu->pose.position.z;
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
  image_transport::Publisher img_pub_;
  image_transport::Publisher img_pub2_;
  ros::Publisher pipe_pub_;
  ros::Publisher ekf_pub_;
  ros::Subscriber alt_sub_;
  ros::Publisher detector_pub;
  //cv_bridge::CvImage cv_image;
  // tf::TransformBroadcaster tf_br_;
public:
  ImageProcessor(ros::NodeHandle &n) : nh_(n),
                                       it_(nh_)
  {
    //img_sub_ = it_.subscribe("/aeroarms_1/camera_0/image_raw", 1, &ImageProcessor::image_callback, this);  //simulation
    img_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageProcessor::image_callback, this);  //real
    //img_sub_ = it_.subscribe("/camera/image", 1, &ImageProcessor::image_callback, this);  // old real
   // sub_alt_ = n.subscribe("/ardrone/navdata", 1000, altitude_Callback);
    img_pub_ = it_.advertise("/output_image", 1);
    img_pub2_ = it_.advertise("/image_detector", 1);
    pipe_pub_ = n.advertise<geometry_msgs::PoseStamped>("/pipe_pose", 1000);
    ekf_pub_ = n.advertise<geometry_msgs::PoseStamped>("/ekf/pipe_pose", 1);
    //alt_sub_ = n.subscribe("/uav_1/mavros/local_position/pose", 1000, IMUCallback); //simulation
    alt_sub_ = n.subscribe("/mavros/local_position/pose", 1000, IMUCallback);   //real
    detector_pub = n.advertise<sensor_msgs::Image>("/Detector_Image",1);

    //pipe_pub_ = n.advertise<geometry_msgs::Twist>("/pipe_pose", 1000);

    // Camera intrinsics
    mIntrinsic << 726.429011, 0.0, 283.809411, // Parrot intrinsic parameters
        0.0, 721.683494, 209.109682,
        0.0, 0.0, 1.0;
  }

  ~ImageProcessor() {}

  void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src = cv_ptr->image;
    t0 = clock();
    
    auto mark = gPiperDetector.detect(src, 0,10000,altitude);
    cv::imshow("display", src);
    cv::waitKey(10);
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
