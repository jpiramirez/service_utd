#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vistarget.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  vistarget vt;

  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("immarkers", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
    pose_pub = nh_.advertise<geometry_msgs::Pose>("/vispose/pose", 1000);
    if(!nh_.getParam("/vispose/targetsize", vt.tgtSize))
	vt.tgtSize = 0.1;
    cout << "Target size is " << vt.tgtSize << endl;
    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
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

    vector<Point2f> markers;
    vt.computeMarkerLocations(cv_ptr->image);
    vt.estimatePose();
    markers = vt.getMarkers();
    if(markers.size() > 0)
	cv::circle(cv_ptr->image, Point(markers[0].x, markers[0].y), 3, CV_RGB(255,0,0));
    for(int i=1; i < markers.size(); i++)
        cv::circle(cv_ptr->image, Point(markers[i].x, markers[i].y), 3, CV_RGB(0,255,0));

    geometry_msgs::Pose tgtPose;

    tgtPose.position.x = vt.position.val[0];
    tgtPose.position.y = vt.position.val[1];
    tgtPose.position.z = vt.position.val[2];

    pose_pub.publish(tgtPose);

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_pose_estimator");
  ImageConverter ic;
  ros::spin();
  return 0;
}
