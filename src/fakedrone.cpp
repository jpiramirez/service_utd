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


class fakedrone
{
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;
  ros::Subscriber ctrl_sub;
  double x, y, z, psi;
  double xdot, ydot, zdot;
  double u1, u2, u3, u4;
  ros::Time ptime, ctime;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  geometry_msgs::Twist control;
  float px, py, pz, ah, pah;
  float setx, sety, setz;
  float errx, erry, errz;
  float perrx, perry, perrz;
  double g, m;

  ros::Timer timer;

public:
  fakedrone()
  {
    ctrl_sub = nh_.subscribe("cmd_vel", 2, &fakedrone::Callback, this);
    string odomtopic;
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("ardrone/pose", 2);
//    point_sub = nh_.subscribe("ardrone/setpoint", 2, &fakedrone::setpointCallback, this);
    ROS_INFO_STREAM("Emulated drone created.");

    vector<double> initstate;
    nh_.getParam("droneinit", initstate);
    x = initstate[0];
    y = initstate[1];
    z = initstate[2];
    psi = initstate[3];

    xdot = 0.0;
    ydot = 0.0;
    zdot = 0.0;
    u1 = 0;
    u2 = 0;
    u3 = 0;
    u4 = 0;
    g = 9.81;
    m = 0.5;

    ptime = ros::Time::now();
    ctime = ros::Time::now();
    while(ptime.toSec() == 0.0 || ctime.toSec() == 0.0)
    {
      ptime = ros::Time::now();
      ctime = ros::Time::now();
    }

    timer = nh_.createTimer(ros::Duration(0.004), &fakedrone::computePose, this);

  }

  ~fakedrone()
  {
  }

  void Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
      u1 = msg->linear.x;
      u2 = msg->linear.y;
      u3 = msg->linear.z;
      u4 = msg->angular.z;
  }

  void computePose(const ros::TimerEvent& te)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

      if(t < 0.004)
        t = 0.004;

      odom.header.stamp = ctime;

      x = x + t*xdot;
      y = y + t*ydot;
      z = z + t*zdot;
      xdot = xdot + t*m*g*(-u2*sin(psi) + u1*cos(psi));
      ydot = ydot + t*m*g*(u2*cos(psi) + u1*sin(psi));
      zdot = zdot + t*u3;

      psi = psi + t*u4;
      if(psi > M_PI)
        psi = -M_PI;
      else if(psi < -M_PI)
        psi = M_PI;

      odom.header.stamp = ctime;
      odom.header.frame_id = "world";
      odom.pose.position.x = x;
      odom.pose.position.y = y;
      odom.pose.position.z = z;
      odom.pose.orientation.w = cos(psi/2.0);
      odom.pose.orientation.z = sin(psi/2.0);

      pose_pub.publish(odom);

      ptime = ctime;
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fakedrone");
  fakedrone dr;
  ros::spin();
  return 0;
}
