// Information based planner or controller
// Needs a set-point controller in place
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <service_utd/ProbMap.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "targetstateestimator.h"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;
using namespace cv;

class infoController
{
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub;
  ros::Publisher map_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber object_sub;
  geometry_msgs::Vector3 wp_msg;
  service_utd::ProbMap pm_msg;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  float px, py, pz;
  float setx, sety, setz;
  targetStateEstimator *tse;
  float fv, fh;
  float av, ah;
  int nsqx, nsqy;
  ros::Timer timer;
  Rect area;
  int seqnum;
  ofstream timestamps;
  float xp, yp;
  
public:
  infoController()
  {
    waypoint_pub = nh_.advertise<geometry_msgs::Vector3>("/ardrone/setpoint", 2);
    map_pub = nh_.advertise<service_utd::ProbMap>("/probmap", 2);
    pose_sub = nh_.subscribe("/ardrone/pose", 2, &infoController::Callback, this);
    object_sub = nh_.subscribe("/objectdetected", 2, &infoController::detectionCallback, this);

    ROS_INFO_STREAM("Set point controller initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    px = 0;
    py = 0;
    pz = 0;
    setx = 0;
    sety = 0;
    setz = 0.8;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    nsqx = 40;
    nsqy = 40;
    tse = new targetStateEstimator(nsqy, nsqx, 0.8, 0.2, 0.05, 0.05);
    // These come from the bottom camera calibration for the ardrone2
    fv = 701.654116;
    fh = 700.490828;
    av = 2.0*atan(180.0/fv);
    ah = 2.0*atan(320.0/fh);
    xp = 0;
    yp = 0;
    seqnum = 0;
    area.x = 0;
    area.y = 0;
    area.width = 0;
    area.height = 0;

    timer = nh_.createTimer(ros::Duration(2.0), &infoController::computeWaypoint, this);
  }

  ~infoController()
  {
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;
  }
  
  void detectionCallback(const std_msgs::Bool::ConstPtr& msg)
  {

      bool detected = msg->data;

      // The camera is offset 2" from the drone center in the x-axis
      float x1 = x - 2.0*0.0254 - xp;
      float x2 = x - 2.0*0.0254 + xp;
      float y1 = y - yp;
      float y2 = y + yp;
      
      
      
      // Our grid is 2x2 meters, centered at (0,0) on the floor
      area.x = floor(nsqx/2-nsqx*y2/2);
      area.y = floor(nsqy/2-nsqy*x2/2);
      xp = z*tan(fabs(av/2.0));
      yp = z*tan(fabs(ah/2.0));
      area.width = floor(nsqx*yp);
      area.height = floor(nsqy*xp);

      tse->updateGrid(area, detected);
      tse->MLE();
      cout << "Mean: " << tse->mean*(-2.0/float(nsqx))+Vec2f(1,1) << endl;
      cout << "StdDev: " << tse->std*(-2.0/float(nsqx))+Vec2f(1,1) << endl;

      pm_msg.data.clear();
      pm_msg.header.seq = seqnum;
      pm_msg.header.stamp = ros::Time::now();
      pm_msg.header.frame_id = "map";
      pm_msg.width = nsqx;
      pm_msg.height = nsqy;
      
      Mat data = tse->getGrid();
      for(int i=0; i < data.rows; i++)
      {
          for(int j=0; j < data.cols; j++)
          {
              pm_msg.data.push_back(data.at<float>(i,j));
          }
      }


      map_pub.publish(pm_msg);

      seqnum++;
  }
  
  void computeWaypoint(const ros::TimerEvent& te)
  {
    // This is the real controller. Once we have a probability distribution for the target
    // location we send the drone to the most informative location. This requires the setpoint
    // controller to be running.

      if (area.width == 0 || area.height == 0)
      {
          return;
      }
    
    Mat M = tse->getGrid();
    Mat mask = Mat::ones(area.height, area.width, CV_32F);
    Mat result = Mat::zeros(M.rows, M.cols, CV_32F);
    filter2D(M, result, -1, mask);
    
    Mat probvol = cv::abs(result - 0.5);
    Point minloc;
    minMaxLoc(probvol, NULL, NULL, &minloc, NULL);
    
    //cout << probvol << endl;
    cout << minloc << endl;
    
    // Converting grid coordinates to world coordinates
    wp_msg.x = -2.0*minloc.y/float(nsqy) + 1.0;
    wp_msg.y = -2.0*minloc.x/float(nsqx) + 1.0;
    wp_msg.z = 0.8;
    waypoint_pub.publish(wp_msg);
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_based_ctrl");
  infoController dr;
  ros::spin();
  return 0;
}
