// Set-point controller for the rover
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <math.h>
#include <iomanip>
#include <utility>
#include <algorithm>
#include <boost/lexical_cast.hpp>


using namespace std;
using namespace Eigen;


class fakerover
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  ros::Subscriber ctrl_sub;
  double x, y, z, theta;
  ros::Time ptime, ctime;
  ros::Duration d;
  nav_msgs::Odometry odom;
  geometry_msgs::Twist control;
  float px, py, pz, ah, pah;
  float setx, sety, setz;
  float errx, erry, errz;
  float perrx, perry, perrz;
  const gsl_rng_type *T;
  gsl_rng *RNG;
  double v, u;

  ros::Timer timer;

public:
  fakerover()
  {
    ctrl_sub = nh_.subscribe("cmd_vel", 2, &fakerover::Callback, this);
    string odomtopic;
    nh_.param<std::string>("odometry_topic", odomtopic, "p3dx/base_pose_ground_truth");
    pose_pub = nh_.advertise<nav_msgs::Odometry>(odomtopic, 2);
//    point_sub = nh_.subscribe("ardrone/setpoint", 2, &fakerover::setpointCallback, this);
    ROS_INFO_STREAM("Emulated rover created.");

    vector<double> initstate;
    nh_.getParam("roverinit", initstate);
    x = initstate[0];
    y = initstate[1];
    z = initstate[2];
    theta = initstate[3];

    v = 0.0;
    u = 0.0;

    ptime = ros::Time::now();
    ctime = ros::Time::now();

    T = gsl_rng_mt19937;
    RNG = gsl_rng_alloc(T);
    gsl_rng_env_setup();

    timer = nh_.createTimer(ros::Duration(0.01), &fakerover::computePose, this);

  }

  ~fakerover()
  {
  }

  void Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
      v = msg->linear.x;
      u = msg->angular.z;

  }

  void computePose(const ros::TimerEvent& te)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

      odom.header.stamp = ctime;

      x = x + t*v*cos(theta);
      y = y + t*v*sin(theta);
      theta = theta + t*u;
      if(theta > M_PI)
        theta = -M_PI;
      else if(theta < -M_PI)
        theta = M_PI;

      odom.header.stamp = ctime;
      odom.header.frame_id = "world";
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation.w = cos(theta/2.0);
      odom.pose.pose.orientation.z = sin(theta/2.0);

      pose_pub.publish(odom);

      ptime = ctime;
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fakerover");
  fakerover dr;
  ros::spin();
  return 0;
}
