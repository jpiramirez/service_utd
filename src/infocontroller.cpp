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
#include <nav_msgs/GridCells.h>
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
  nav_msgs::GridCells pm_msg;
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
  
public:
  infoController()
  {
    waypoint_pub = nh_.advertise<geometry_msgs::Vector3>("/ardrone/setpoint", 2);
    map_pub = nh_.advertise<nav_msgs::GridCells>("/probmap", 2);
    pose_sub = nh_.subscribe("/ardrone/pose", 2, &infoController::Callback, this);
    object_sub = nh_.subscribe("/objectdetected", 2, &infoController::detectionCallback, this);
    timer = nh_.createTimer(ros::Duration(2.0), &infoController::computeWaypoint, this);
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
    seqnum = 0;
    timestamps.open("timestamps.txt", ios::out);
  }

  ~infoController()
  {
    
    FileStorage fs("delme.xml", FileStorage::WRITE);
    fs << "G" << tse->getGrid();
    fs.release();
    timestamps.close();
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;
  }
  
  void detectionCallback(const std_msgs::Bool::ConstPtr& msg)
  {
      float xp = z*tan(fabs(av/2.0));
      float yp = z*tan(fabs(ah/2.0));
      bool detected = msg->data;

      // The camera is offset 2" from the drone center in the x-axis
      float x1 = x - 2.0*0.0254 - xp;
      float x2 = x - 2.0*0.0254 + xp;
      float y1 = y - yp;
      float y2 = y + yp;
      
      
      
      // Our grid is 2x2 meters, centered at (0,0) on the floor
      area.x = floor(nsqx/2-nsqx*y2/2);
      area.width = floor(nsqx*yp);
      area.y = floor(nsqy/2-nsqy*x2/2);
      area.height = floor(nsqy*xp);
      tse->updateGrid(area, detected);
      tse->MLE();
      cout << "Mean: " << tse->mean*(-2.0/float(nsqx))+Vec2f(1,1) << endl;
      cout << "StdDev: " << tse->std*(-2.0/float(nsqx))+Vec2f(1,1) << endl;

      pm_msg.cells.clear();
      pm_msg.header.seq = seqnum;
      pm_msg.header.stamp = ros::Time::now();
      pm_msg.header.frame_id = "map";
      geometry_msgs::Point pt;
      
      char name[80];
      sprintf(name, "%04i", seqnum);
      string filename(name);
      ofstream fs;
      filename = "grid" + filename + ".txt";
      fs.open(filename.c_str(), ios::out);
      Mat data = tse->getGrid();
      float tmp;
      for(int i=0; i < data.rows; i++)
      {
          for(int j=0; j < data.cols; j++)
          {
              fs << data.at<float>(i,j) << " " ;
              pt.z = data.at<float>(i,j);
              pm_msg.cells.push_back(pt);
          }
          fs << endl;
      }
      fs.close();

      // Timestamp storage for visualization
      ros::Time ctime;
      ctime = ros::Time::now();
      timestamps << ctime.sec;
      timestamps << setfill('0') << setw(9) << ctime.nsec;
      timestamps << "," << seqnum << endl;

      map_pub.publish(pm_msg);

      seqnum++;
  }
  
  void computeWaypoint(const ros::TimerEvent& te)
  {
    // This is the real controller. Once we have a probability distribution for the target
    // location we send the drone to the most informative location. This requires the setpoint
    // controller to be running.
    
    Mat M = tse->getGrid();
    Mat mask = Mat::ones(area.height, area.width, CV_32F);
    Mat result = Mat::zeros(M.rows, M.cols, CV_32F);
    filter2D(M, result, -1, mask);
    
    Mat probvol = cv::abs(result - 0.5);
    Point minloc;
    minMaxLoc(probvol, NULL, NULL, &minloc, NULL);
    
    //cout << probvol << endl;
    cout << minloc << endl;
    
    wp_msg.x = -2.0*minloc.y/float(nsqy) + 1.0;
    wp_msg.y = -2.0*minloc.x/float(nsqx) + 1.0;
    wp_msg.z = 0.8;
    waypoint_pub.publish(wp_msg);
    //char name[80];
    //sprintf(name, "%04i", seqnum);
    //string filename(name);
    //FileStorage fs("grid" + filename + ".xml", FileStorage::WRITE);
    //fs << "G" << tse->getGrid();
    //fs.release();
    //seqnum++;
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_based_ctrl");
  infoController dr;
  ros::spin();
  return 0;
}
