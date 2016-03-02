// Finding a target as a color blob
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class fakedetector
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  ros::Publisher tgtpose_pub;
  ros::Subscriber caminfo_sub;
  ros::Subscriber campose_sub;
  ros::Subscriber targetpose_sub;

  ros::Time ctime, ptime;
  ros::Timer timer;
  Vector3d camerapose;
  double tgtSize;

  double tx, ty, tz;


public:
  fakedetector()
  {
    pose_pub = nh_.advertise<std_msgs::Bool>("objectdetected", 2);
    campose_sub = nh_.subscribe("ardrone/pose", 2, &fakedetector::poseCallback, this);
    tgtpose_pub = nh_.advertise<geometry_msgs::Pose>("objectpose", 1);
    nh_.param("targetdetect/targetsize", tgtSize, 0.1);
    ROS_INFO_STREAM("Target size is " << tgtSize);

    std::string targettopic;
    nh_.param<std::string>("target_topic", targettopic, "/target/p3dx/base_pose_ground_truth");
    targetpose_sub = nh_.subscribe(targettopic, 2, &fakedetector::targetPoseCallback, this);

    timer = nh_.createTimer(ros::Duration(0.05), &fakedetector::pubinfo, this);
    camerapose = Vector3d(0, 0, 0);
    tx = 0; ty = 0; tz = 0;

    ctime = ros::Time::now();
    ptime = ros::Time::now();
  }

  ~fakedetector()
  {
  }

  void targetPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      tx = msg->pose.pose.position.x;
      ty = msg->pose.pose.position.y;
      tz = msg->pose.pose.position.z;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      camerapose[0] = msg->pose.position.x;
      camerapose[1] = msg->pose.position.y;
      camerapose[2] = msg->pose.position.z;
  }

  void pubinfo(const ros::TimerEvent& te)
  {
      std_msgs::Bool tfound;

      double dist = pow(tx-camerapose[0], 2.0);
      dist += pow(ty-camerapose[1], 2.0);
      dist = sqrt(dist);
      if(dist < tgtSize)
        tfound.data = true;
      else
        tfound.data = false;

      pose_pub.publish(tfound);

      if(tfound.data == true)
      {
        geometry_msgs::Pose tgtpose;
        tgtpose.position.x = tx - camerapose[0];
        tgtpose.position.y = ty - camerapose[1];
        tgtpose_pub.publish(tgtpose);
      }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_detector");
  fakedetector ic;
  ros::spin();
  return 0;
}
