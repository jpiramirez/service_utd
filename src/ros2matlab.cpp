// The purpose of this program is to translate all the messages
// related to the sensor platform into a format that is easy to
// parse with Matlab or similar tools.
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016

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

class ros2matlab
{
  ros::NodeHandle nh_;
  ros::Subscriber map_sub;
  ros::Subscriber searchmap_sub;
  ros::Subscriber pose_sub, tgtpose_sub, tgtdetect_sub;
  ros::Subscriber object_sub;
  geometry_msgs::Vector3 wp_msg;
  service_utd::ProbMap pm_msg;
  float x, y, z, tx, ty, tz;
  ros::Time ptime, ctime;
  ros::Duration d;
  targetStateEstimator *tse;
  float fv, fh;
  float av, ah;
  int nsqx, nsqy;
  Rect area;
  int seqnum, sseqnum;
  ofstream poses;
  ofstream sposes;
  bool tgtDetectedFlag;
  
public:
  ros2matlab()
  {
    map_sub = nh_.subscribe<service_utd::ProbMap>("/probmap", 2, &ros2matlab::mapCallback, this);
    searchmap_sub = nh_.subscribe<service_utd::ProbMap>("/searchmap", 2, &ros2matlab::searchmapCallback, this);
    pose_sub = nh_.subscribe("/ardrone/pose", 2, &ros2matlab::poseCallback, this);
    tgtpose_sub = nh_.subscribe("/target/pose", 2, &ros2matlab::tgtposeCallback, this);
    tgtdetect_sub = nh_.subscribe("/objectdetected", 1, &ros2matlab::tgtdetectCallback, this);
    ROS_INFO_STREAM("Translating ROS messages into a parseable Matlab file");
    seqnum = 0;
    sseqnum = 0;
    x = 0;
    y = 0;
    z = 0;
    tx = 0;
    ty = 0;
    tz = 0;
    tgtDetectedFlag = false;
    poses.open("poses.txt", ios::out);
    sposes.open("searchposes.txt", ios::out);
  }

  ~ros2matlab()
  {
    poses.close();
    sposes.close();
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;
  }

  void tgtposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      tx = msg->pose.position.x;
      ty = msg->pose.position.y;
      tz = msg->pose.position.z;
  }

  void tgtdetectCallback(const std_msgs::Bool::ConstPtr& msg)
  {
      tgtDetectedFlag = msg->data;
  }
  
  void mapCallback(const service_utd::ProbMap::ConstPtr& msg)
  {
      
      char name[80];
      sprintf(name, "%04i", seqnum);
      string filename(name);
      ofstream fs;
      filename = "grid" + filename + ".txt";
      fs.open(filename.c_str(), ios::out);
      int count = 0;
      for(int i=0; i < msg->height; i++)
      {
          for(int j=0; j < msg->width; j++)
          {
              fs << msg->data[count] << " " ;
              count++;
          }
          fs << endl;
      }
      fs.close();

      // Timestamp storage for visualization
      ros::Time ctime;
      ctime = ros::Time::now();
      poses << ctime.sec;
      poses << setfill('0') << setw(9) << ctime.nsec;
      poses << "," << seqnum << "," << x << "," << y << "," << z;
      poses << "," << tx << "," << ty << "," << tz << ",";
      if(tgtDetectedFlag)
          poses << 1;
      else
          poses << 0;
      poses << endl;

      seqnum++;
  }

  void searchmapCallback(const service_utd::ProbMap::ConstPtr& msg)
  {

      char name[80];
      sprintf(name, "%04i", sseqnum);
      string filename(name);
      ofstream fs;
      filename = "searchgrid" + filename + ".txt";
      fs.open(filename.c_str(), ios::out);
      int count = 0;
      for(int i=0; i < msg->height; i++)
      {
          for(int j=0; j < msg->width; j++)
          {
              fs << msg->data[count] << " " ;
              count++;
          }
          fs << endl;
      }
      fs.close();

      // Timestamp storage for visualization
      ros::Time ctime;
      ctime = ros::Time::now();
      sposes << ctime.sec;
      sposes << setfill('0') << setw(9) << ctime.nsec;
      sposes << "," << sseqnum << "," << x << "," << y << "," << z;
      sposes << "," << tx << "," << ty << "," << tz << ",";
      if(tgtDetectedFlag)
          sposes << 1;
      else
          sposes << 0;
      sposes << endl;

      sseqnum++;
  }
  
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros2matlabtranslate");
  ros2matlab dr;
  ros::spin();
  return 0;
}
