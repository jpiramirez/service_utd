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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera_calibration_parsers/parse.h>
#include "colortarget.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber caminfo_sub;
  colortarget vt;

  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("immarkers", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
    pose_pub = nh_.advertise<std_msgs::Bool>("/objectdetected", 2);
    nh_.param("/vispose/targetsize", vt.tgtSize, 0.1);
    ROS_INFO_STREAM("Target size is " << vt.tgtSize);

    std::string calibfile;
    nh_.param<std::string>("/vispose/camera_info_url", calibfile, "calibration.yml");
	std::string colordef;
	nh_.param<std::string>("/vispose/color", colordef, "0 71 213");
	std::stringstream ss(colordef);
	ss >> vt.color.val[0];
	ss >> vt.color.val[1];
	ss >> vt.color.val[2];
	
	ROS_INFO_STREAM("Using color " << vt.color.val[0] << " " << \
	                vt.color.val[1] << " " << vt.color.val[2]);
	
    std::string camname;
    sensor_msgs::CameraInfo caminfo;

    if(!camera_calibration_parsers::readCalibration(calibfile, camname, caminfo))
        exit(1);
    
    // There was a way to do the following assignments in a more compact manner
    // but I just can't remember
    vt.calibMatrix.at<double>(0,0) = caminfo.K[0];
    vt.calibMatrix.at<double>(0,1) = caminfo.K[1];
    vt.calibMatrix.at<double>(0,2) = caminfo.K[2];
    vt.calibMatrix.at<double>(1,0) = caminfo.K[3];
    vt.calibMatrix.at<double>(1,1) = caminfo.K[4];
    vt.calibMatrix.at<double>(1,2) = caminfo.K[5];
    vt.calibMatrix.at<double>(2,0) = caminfo.K[6];
    vt.calibMatrix.at<double>(2,1) = caminfo.K[7];
    vt.calibMatrix.at<double>(2,2) = caminfo.K[8];

    for(int i=0; i < 5; i++)
	vt.distCoeff.push_back(caminfo.D[i]);

    ROS_INFO_STREAM("Calibration matrix: " << vt.calibMatrix);

  }

  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
	cv_ptr->image.convertTo(fimage, CV_32FC3);
    vt.computeStateWithColorBlob(fimage);
 	cv::circle(cv_ptr->image, vt.tgtLoc, 3, CV_RGB(255,0,0));
    cv::rectangle(cv_ptr->image, vt.tgtRect, CV_RGB(0,255,0), 3);

    geometry_msgs::Pose tgtPose;
    std_msgs::Bool tfound;

    if(vt.targetFound())
    {
        tgtPose.position.x = vt.position.val[0];
        tgtPose.position.y = vt.position.val[1];
        tgtPose.position.z = vt.position.val[2];
    
        tgtPose.orientation.w = vt.orientation.val[0];
        tgtPose.orientation.x = vt.orientation.val[1];
        tgtPose.orientation.y = vt.orientation.val[2];
        tgtPose.orientation.z = vt.orientation.val[3];

	tfound.data = true;
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
