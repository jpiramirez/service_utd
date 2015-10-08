// Finding a target as a color blob
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <camera_calibration_parsers/parse.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  ros::Publisher tgtpose_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber caminfo_sub;
  ros::Subscriber campose_sub;

  Vec3i color;
  Vec3f camerapose;

  SimpleBlobDetector *detector;
  SimpleBlobDetector::Params params;
  vector<KeyPoint> keypoints;

  double tgtSize;
  Mat calibMatrix;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("immarkers", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
    pose_pub = nh_.advertise<std_msgs::Bool>("objectdetected", 2);
    campose_sub = nh_.subscribe("ardrone/pose", 2, &ImageConverter::poseCallback, this);
    tgtpose_pub = nh_.advertise<geometry_msgs::Pose>("objectpose", 1);
    nh_.param("targetdetect/targetsize", tgtSize, 0.1);
    ROS_INFO_STREAM("Target size is " << tgtSize);

    std::string calibfile;
    nh_.param<std::string>("targetdetect/camera_info_url", calibfile, "calibration.yml");
	std::string colordef;
    nh_.param<std::string>("targetdetect/color", colordef, "0 71 213");
	std::stringstream ss(colordef);
    ss >> color[0];
    ss >> color[1];
    ss >> color[2];

	
    ROS_INFO_STREAM("Using color " << color[0] << " " << \
                    color[1] << " " << color[2]);
	
    std::string camname;
    sensor_msgs::CameraInfo caminfo;

    if(!camera_calibration_parsers::readCalibration(calibfile, camname, caminfo))
        exit(1);
    
    // There was a way to do the following assignments in a more compact manner
    // but I just can't remember
    calibMatrix.create(3, 3, CV_64FC1);
    calibMatrix.at<double>(0,0) = caminfo.K[0];
    calibMatrix.at<double>(0,1) = caminfo.K[1];
    calibMatrix.at<double>(0,2) = caminfo.K[2];
    calibMatrix.at<double>(1,0) = caminfo.K[3];
    calibMatrix.at<double>(1,1) = caminfo.K[4];
    calibMatrix.at<double>(1,2) = caminfo.K[5];
    calibMatrix.at<double>(2,0) = caminfo.K[6];
    calibMatrix.at<double>(2,1) = caminfo.K[7];
    calibMatrix.at<double>(2,2) = caminfo.K[8];

    ROS_INFO_STREAM("Calibration matrix: " << calibMatrix);


    params.filterByCircularity = true;
    params.minCircularity = 0.55;
    params.maxCircularity = 1.0;
    params.filterByArea = true;
    params.minArea = 50;
    params.maxArea = 640*480;
    params.filterByInertia = false;
    params.minInertiaRatio = 0.5;
    params.maxInertiaRatio = 0.75;
    params.filterByColor = false;
    params.blobColor = 255;
    params.filterByConvexity = false;
    params.minThreshold = 250;
    params.maxThreshold = 255;
    params.thresholdStep = 1;
    params.minRepeatability = 3;
    params.minDistBetweenBlobs = 100;

    detector = new SimpleBlobDetector(params);

    camerapose = Vec3f(0, 0, 0);
  }

  ~ImageConverter()
  {
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      camerapose[0] = msg->pose.position.x;
      camerapose[1] = msg->pose.position.y;
      camerapose[2] = msg->pose.position.z;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    int i, j;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	Mat fimage;
    cv_ptr->image.convertTo(fimage, CV_8UC3);
    Mat grayscale(fimage.rows, fimage.cols, CV_8UC1);
    //vt.computeStateWithColorBlob(fimage);
    //cv::circle(cv_ptr->image, vt.tgtLoc, 3, CV_RGB(255,0,0));
    //cv::rectangle(cv_ptr->image, vt.tgtRect, CV_RGB(0,255,0), 3);

    geometry_msgs::Pose tgtPose;
    std_msgs::Bool tfound;
    double colorDist;

    for(i=0; i < fimage.rows; i++)
        for(j=0; j < fimage.cols; j++)
        {
            colorDist = 0.0;
            for(int k=0; k < 3; k++)
                colorDist += pow(fimage.at<Vec3b>(i,j)[k]-color[k], 2.0f);
            colorDist = sqrt(colorDist);
            if(colorDist < 200)
                grayscale.at<uchar>(i,j) = 255;
            else
                grayscale.at<uchar>(i,j) = 0;
        }

    //Mat grayscale(fimage.rows, fimage.cols, CV_8UC1);
    //cvtColor(fimage, grayscale, CV_RGB2GRAY);
    //grayscale.convertTo(grayscale, CV_8UC1);
    //grayscale.convertTo(cv_ptr->image, CV_8UC3);
    cvtColor(grayscale, cv_ptr->image, CV_GRAY2BGR);

    keypoints.clear();
    detector->detect(grayscale, keypoints);
    if(keypoints.size() == 0)
        cout << "No blobs detected" << endl;
    else
        cout << "Blobs detected" << keypoints.size() << endl;

    Point pt;
    for(int i=0; i < keypoints.size(); i++)
    {
        pt.x = keypoints[i].pt.x;
        pt.y = keypoints[i].pt.y;
        cv::circle(cv_ptr->image, pt, 20, CV_RGB(0,255,0), -1);
    }

    //drawKeypoints(grayscale, keypoints, cv_ptr->image, CV_RGB(255, 0, 0));

    bool targetFound = false;
    if(keypoints.size() > 0)
        targetFound = true;

    double f = calibMatrix.at<double>(0, 0);
    if(targetFound)
    {
        // The target position is being given relative to the drone pose

//        tgtPose.position.z = tgtSize*f/(2.0*keypoints[0].size);
        tgtPose.position.z = camerapose[2];
        tgtPose.position.x = -(pt.y - fimage.rows/2.0)/f;
        tgtPose.position.y = -(pt.x - fimage.cols/2.0)/f;
        tgtPose.position.x *= tgtPose.position.z;
        tgtPose.position.y *= tgtPose.position.z;
        cout << keypoints[0].size << endl;
    
        tgtPose.orientation.w = 1;
        tgtPose.orientation.x = 0;
        tgtPose.orientation.y = 0;
        tgtPose.orientation.z = 0;

        tfound.data = true;
        tgtpose_pub.publish(tgtPose);
    }
    else
        tfound.data = false;
      
    pose_pub.publish(tfound);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}
