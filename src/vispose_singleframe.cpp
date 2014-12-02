// Pose estimation using fiducials.
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera_calibration_parsers/parse.h>
#include "vistarget.h"

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_pose_estimator");

    ros::NodeHandle nh_;
    ros::Publisher pose_pub;
    
    ros::Subscriber caminfo_sub;
    vistarget vt;
    Mat image, orimage;

    
    pose_pub = nh_.advertise<geometry_msgs::Pose>("/vispose/pose", 2);
    nh_.param("/vispose/targetsize", vt.tgtSize, 0.1);
    ROS_INFO_STREAM("Target size is " << vt.tgtSize);
    vt.setTargetCoords();

    std::string calibfile;
    nh_.param<std::string>("/vispose/camera_info_url", calibfile, "calibration.yml");
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

    vector<Point2f> markers;
    ros::Rate up_rate(1);
    orimage = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    while(ros::ok())
    {
	image = orimage.clone();
        vt.computeMarkerLocations(image);
        vt.estimatePose();
        markers = vt.getMarkers();
        if(markers.size() > 0)
    	cv::circle(image, Point(markers[0].x, markers[0].y), 3, CV_RGB(255,0,0));
        for(int i=1; i < markers.size(); i++)
            cv::circle(image, Point(markers[i].x, markers[i].y), 3, CV_RGB(0,255,0));
    
        geometry_msgs::Pose tgtPose;
    
        if(vt.targetFound())
        {
            tgtPose.position.x = vt.position.val[0];
            tgtPose.position.y = vt.position.val[1];
            tgtPose.position.z = vt.position.val[2];
        
            tgtPose.orientation.w = vt.orientation.val[0];
            tgtPose.orientation.x = vt.orientation.val[1];
            tgtPose.orientation.y = vt.orientation.val[2];
            tgtPose.orientation.z = vt.orientation.val[3];
    	    up_rate.sleep();
            pose_pub.publish(tgtPose);
        }
    
    
        ros::spinOnce();
    }
    return 0;
}
