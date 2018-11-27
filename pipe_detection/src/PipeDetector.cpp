//
//
//
//
//


#include "PipeDetector.h"

using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
PipeDetector::PipeDetector(std::string _cfgModel, std::string weights){
    //img_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageProcessor::image_callback, this);  //real
    img_sub_ = it_.subscribe("/camera/image", 1, &PipeDetector::colorImageCallback, this);  // old real
    img_depth_sub_ = it_.subscribe("/camera/image_depth", 1, &PipeDetector::depthImageCallback, this);
    img_pub_ = it_.advertise("/output_image_bw", 1);
    img_pub2_ = it_.advertise("/image_detector", 1);
    pipe_pub_ = n.advertise<geometry_msgs::PoseStamped>("/pipe_pose", 1000);
    ekf_pub_ = n.advertise<geometry_msgs::PoseStamped>("/ekf/pipe_pose", 1);
    alt_sub_ = n.subscribe("/mavros/local_position/pose", 1000, &PipeDetector::IMUCallback, this);   //real
    detector_pub = n.advertise<sensor_msgs::Image>("/Detector_Image",1);

    //pipe_pub_ = n.advertise<geometry_msgs::Twist>("/pipe_pose", 1000);

    // Camera intrinsics
    mIntrinsic << 617.8070678710938, 0.0, 326.96380615234375, // Parrot intrinsic parameters
        0.0, 617.9169311523438, 241.34239196777344,
        0.0, 0.0, 1.0;

    if(!detector.init(_argv[1], _argv[2])){
        std::cout << "Failed initialization of network" << std::endl;
        return false;
    }
}

//---------------------------------------------------------------------------------------------------------------------
void PipeDetector::IMUCallback(const geometry_msgs::PoseStamped::ConstPtr& _imu) {
    altitude = _imu->pose.position.z;
    q.x() = _imu->pose.orientation.x;
    q.y() = _imu->pose.orientation.y;
    q.z() = _imu->pose.orientation.z;
    q.w() = _imu->pose.orientation.w;
}

//---------------------------------------------------------------------------------------------------------------------
void PipeDetector::depthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg){
    cv_bridge::CvImagePtr cv_ptr_depth;
    image_depth_msg = *depth_msg;  
    
    try {
      cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    current_depth_msg = cv_ptr_depth->image; 
}

//---------------------------------------------------------------------------------------------------------------------
void PipeDetector::colorImageCallback(const sensor_msgs::ImageConstPtr &msg){
    // Decode image from message
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src = cv_ptr->image;

    // YOLO detection
    auto detections = detector.detect(src);
    if(detections.size() == 0)
        return;
    cv::Rect rec(x_min = detections[0][2], // xmin
                 y_min = detections[0][3], // ymin
                 width = detections[0][4] - detections[0][2], //width
                 height = detections[0][5] - detections[0][3]); //height
    cv::Mat portionOfImage = src(rec);

    imshow("Portion image", portionOfImage);
    
    // Resize, blur and meanshift
    Mat res, src2;
    GaussianBlur(portionOfImage, src2, Size(-1,-1), 2, 2);
    pyrMeanShiftFiltering(src2, res, 4, 25, 1);

    // Color cluster space
    cv::cvtColor(res, dst, CV_BGR2HSV);
    std::vector<rgbd::ImageObject> objects;
    rgbd::ColorClusterSpace *ccs = rgbd::createSingleClusteredSpace(
        0, 180,
        50, 255, 
        50, 255,
        180, 255, 255,
        32);

    rgbd::ColorClustering<uchar>(dst.data,
                                 dst.cols,
                                 dst.rows,
                                 5000,  // minimun number of pixels detected
                                 objects,
                                 *ccs);


    // Detect contours using canny
    Mat dst2, cdst2;
    Canny(res, dst2, 50, 150, 3);
    imshow("canny", dst2);
    vector<vector<Point> > contours2;
    vector<Vec4i> hierarchy2;
    Mat drawing;

    // Complete contours
    completeContours(dst2);

    // Dilate and find poly contours
    cv::dilate(dst2, dst2, cv::Mat(), cv::Point(-1,-1),1);

    findContours( dst2, contours2, hierarchy2, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( dst2.size(), CV_8UC3 );

    vector<Point> approxShape;
    for(size_t i = 0; i < contours2.size(); i++) {
        approxPolyDP(contours2[i], approxShape, arcLength(Mat(contours2[i]), true)*0.04, true);
        drawContours(drawing, contours2, i, Scalar(255, 255, 255), CV_FILLED);   // fill White
    }
    
    cv::cvtColor(dst, dst, CV_HSV2BGR);
    cv::Mat display = src2.clone();
    for (auto &ob : objects) {
      cv::Rect bb(
          ob.centroid().x - ob.width() / 2,
          ob.centroid().y - ob.height() / 2,
          ob.width(),
          ob.height());
      cv::rectangle(display, bb, cv::Scalar(0, 255, 0), 2);
    }

    // Publish result
    cv_bridge::CvImage send(cv_ptr->header, cv_ptr->encoding, dst);
    img_pub_.publish(send.toImageMsg());

    /////////// PCA
    Mat gray;
    cvtColor(dst, gray, COLOR_BGR2GRAY);
    Mat bw;
    threshold(gray, bw, 50, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    Mat combined_image( bw.size(), bw.type() );
    for( int y = 0; y < bw.rows; y++ ) {
        for( int x = 0; x < bw.cols; x++ ) {
            if (bw.at<uchar>(y,x) == 255 && drawing.at<Vec3b>(y,x)[0] == 255)
                combined_image.at<uchar>(y,x) = 255;
            else
                combined_image.at<uchar>(y,x) = 0;
            }
    }

    cv::erode(combined_image, combined_image, cv::Mat(), cv::Point(-1,-1), 4);
    cv::dilate(combined_image, combined_image, cv::Mat(), cv::Point(-1,-1), 3);

    imshow ("Combined_image", combined_image);

    // 
    float sumatory = 0;
    float num_pixel = 0;
    for (int row = 0; row < combined_image.rows; row++) {
        for (int col = 0; col < combined_image.cols; col++) {
            if(combined_image.at<uchar>(row,col) == 255) {
                if (current_depth_msg.at<uint16_t>(row + y_min, col + x_min) != 0) {
                  sumatory += current_depth_msg.at<uint16_t>(row + y_min, col + x_min);
                  num_pixel++;
                }
            }
        }
    }

    float depth_alt_med = 0; 
    int marcador = 0;
    if (num_pixel > 0) {        
      depth_alt_med = (sumatory/num_pixel)*1.0000000474974513;    //e-03;
      cout << "Pixeles: " << num_pixel << std::endl;
      cout << "Altura_media: " << depth_alt_med << std::endl;
      for (int row = 0; row < combined_image.rows; row++) {
        for (int col = 0; col < combined_image.cols; col++){   
          if(combined_image.at<uint16_t>(row,col) == 255) {
                if (current_depth_msg.at<uint16_t>(row + y_min, col + x_min) > depth_alt_med){ 
                    // cout << "Eliminando suelo" << std::endl;
                    combined_image.at<uchar>(row,col) = 0;
                    marcador += 1;
                }
            }
          // else 
          //       if (current_depth_msg.at<uint16_t>(length + x_min, width + y_min) != 0 && current_depth_msg.at<uint16_t>(length + x_min, width + y_min) < depth_alt_med*1000)
          //           combined_image.at<uchar>(length,width) = 255;
        }
      }
    }
    cout << "Puntos eliminados: " << marcador << std::endl;
    imshow ("Combined_image_depth", combined_image);

    // Find all the contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    findContours(combined_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); ++i) {
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
        m = Eigen::AngleAxisf(euler[0] - M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(euler[1] - M_PI, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf((yaw + M_PI / 2), Eigen::Vector3f::UnitZ()); // changing value of yaw so that the position is in 0º
        //cout << "Angles: " << euler[0]-M_PI << ", " << euler[1]-M_PI << ", " << yaw+M_PI/2 << endl;
        // transform to quaternion
        Eigen::Quaternionf quaternion(m);
        // Initializaing pose
        geometry_msgs::PoseStamped pipe_data;
        geometry_msgs::PoseStamped ekf_pipe_data;
        // xi=fx*x/z+cx, yi=fy*y/z+cy
        pipe_data.pose.position.x = (altitude) * (pipe_center.x - mIntrinsic(0, 2)) / mIntrinsic(0, 0); // x=z*(xi-cx)/fx
        pipe_data.pose.position.y = (altitude) * (pipe_center.y - mIntrinsic(1, 2)) / mIntrinsic(1, 1); // y=z*(yi-cy)/fy
        pipe_data.pose.position.z = altitude;
        pipe_data.pose.orientation.x = 0;
        pipe_data.pose.orientation.y = 0;
        pipe_data.pose.orientation.z = yaw + 3.14159265 / 2;
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
    cv::waitKey(3);
    cv_bridge::CvImage send_src(cv_ptr->header, cv_ptr->encoding, src);
    img_pub2_.publish(send_src.toImageMsg());
    t3 = clock();
}


//---------------------------------------------------------------------------------------------------------------------
void PipeDetector::completeContours(cv::Mat &_image){

    int flag2 = 0;
    for (int y = 0; y < _image.cols; y++) {
        int x = 0;
        if (_image.at<uchar>(x, y) == 255)
            flag2 = 1;
    }
    if (flag2 == 1) {
        for (int y = 1; y < _image.cols - 1; y++) {
            int x = 0;
            _image.at<uchar>(x, y) = 255;
        }
    }

    flag2 = 0;
    for (int x = 0; x < _image.rows; x++) {
        int y = _image.cols - 1;

        if (_image.at<uchar>(x, y) == 255)
            flag2 = 1;
    }

    if (flag2 == 1) {
        for (int x = 1; x < _image.rows - 1; x++) {
            int y = _image.cols - 1;
            _image.at<uchar>(x, y) = 255;
        }
    }

    flag2 = 0;
    for (int y = 0; y < _image.cols; y++) {
        int x = _image.rows - 1;

        if (_image.at<uchar>(x, y) == 255)
            flag2 = 1;
    }

    if (flag2 == 1) {
        for (int y = 1; y < _image.cols - 1; y++) {
            int x = _image.rows - 1;
            _image.at<uchar>(x, y) = 255;
        }
    }

    flag2 = 0;
    for (int x = 0; x < _image.rows; x++) {
        int y = 0;

        if (_image.at<uchar>(x, y) == 255)
            flag2 = 1;
    }

    if (flag2 == 1) {
        for (int x = 1; x < _image.rows - 1; x++) {
            int y = 0;
            _image.at<uchar>(x, y) = 255;
        }
    }
}
