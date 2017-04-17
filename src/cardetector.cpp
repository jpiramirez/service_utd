// Finding a car in a scene
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <camera_calibration_parsers/parse.h>
#include "CarDetector.h"

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

  Scalar color;

  CarDetector *cd;

  double tgtSize;
  Mat calibMatrix;
  bool targetFound;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("immarkers", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
    pose_pub = nh_.advertise<std_msgs::Bool>("objectdetected", 2);
    tgtpose_pub = nh_.advertise<geometry_msgs::Pose>("objectpose", 1);
    nh_.param("targetdetect/targetsize", tgtSize, 0.1);
    ROS_INFO_STREAM("Target size is " << tgtSize);

    std::string calibfile;
    nh_.param<std::string>("targetdetect/camera_info_url", calibfile, "calibration.yml");
    std::string colordef;
    nh_.param<std::string>("targetdetect/color", colordef, "0 255 0");
    std::stringstream ss(colordef);
    ss >> color[0];
    ss >> color[1];
    ss >> color[2];

    Mat origcolor(1,1,CV_8UC3);
    origcolor.at<Vec3b>(0,0)[0] = color[0];
    origcolor.at<Vec3b>(0,0)[1] = color[1];
    origcolor.at<Vec3b>(0,0)[2] = color[2];
    Mat labcolor = origcolor.clone();

    cvtColor(origcolor, labcolor, CV_BGR2Lab);
    color = Scalar(labcolor.at<Vec3b>(0,0));
    ROS_INFO_STREAM("Using color " << color[0] << " " << \
                    color[1] << " " << color[2]);
    color = Scalar(105, 84, 165);



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

    string cdVert;
    nh_.param<std::string>("targetdetect/vertical_cascade", cdVert, "CascadeVertical.xml");
    string cdHoriz;
    nh_.param<std::string>("targetdetect/horizontal_cascade", cdHoriz, "CascadeHorizontal.xml");
    string cdDiag;
    nh_.param<std::string>("targetdetect/diagonal_cascade", cdDiag, "CascadeDiagonal.xml");

    cd = new CarDetector(cdHoriz, cdDiag, cdVert);
    targetFound = false;

  }

  ~ImageConverter()
  {
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

    targetFound = cd->DetectCar(fimage, color, 50);

    double f = calibMatrix.at<double>(0, 0);
    if(targetFound)
    {
        // The target position is being given relative to the drone pose
        Rect crect;
        crect.x = cd->GetX();
        crect.y = cd->GetY();
        crect.width = cd->GetWIDTH();
        crect.height = cd->GetHEIGHT();
        cv::rectangle(cv_ptr->image, crect, CV_RGB(0,255,0), 3);

        tgtPose.position.z = tgtSize*f/(2.0*cd->GetWIDTH());
        tgtPose.position.x = -(cd->GetX() - fimage.rows/2.0)/f;
        tgtPose.position.y = -(cd->GetY() - fimage.cols/2.0)/f;
        tgtPose.position.x *= tgtPose.position.z;
        tgtPose.position.y *= tgtPose.position.z;

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

//    cv_ptr->image = fimage.clone();
    image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}
