// Odometry for the drone
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
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

class droneController
{
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber point_sub;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  geometry_msgs::Twist control;
  float px, py, pz;
  float setx, sety, setz;
  
public:
  droneController()
  {
    ctrl_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    pose_sub = nh_.subscribe("/ardrone/pose", 2, &droneController::Callback, this);
    point_sub = nh_.subscribe("/ardrone/setpoint", 2, &droneController::setpointCallback, this);
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
  }

  ~droneController()
  {
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();
      
      // This code assumes that the order in which the markers arrive is 
      // front    back    left    right
      
      //odom = msg;
      float KPx = 0.3;
      float KPy = 0.3;
      float KPz = 0.3;
      
      float KDx = 0;
      float KDy = 0;
      float KDz = 0;
      
      float Ky = 5;
      
      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;
      
      double angle = 2.0*asin(msg->pose.orientation.z);
      
      //if(fabs(angle) < 1e-2)
	//angle = 0;
      if(fabs(angle) < 5e-1)
      {
	control.linear.x = -KPx*(x-setx) - KDx*((x-setx)-px)/t;
	control.linear.y = -KPy*(y-sety) - KDy*((y-sety)-py)/t;
	control.linear.z = -KPz*(z-setz) - KDz*((z-setz)-pz)/t;
      }
      else
      {
	control.linear.x = 0;
	control.linear.y = 0;
	control.linear.z = 0;
      }
      control.angular.x = 0;
      control.angular.y = 0;
      control.angular.z = -Ky*angle;
      
      
      // Saturation
      if(fabs(control.linear.x) > 0.05)
	control.linear.x = copysign(0.05, control.linear.x);
      if(fabs(control.linear.y) > 0.05)
	control.linear.y = copysign(0.05, control.linear.y);
      if(fabs(control.linear.z) > 0.05)
	control.linear.z = copysign(0.05, control.linear.z);

      // Hover
      if(fabs(control.linear.x) < 1e-2)
	control.linear.x = 0;
      if(fabs(control.linear.y) < 1e-2)
	control.linear.y = 0;
      if(fabs(control.linear.z) < 1e-2)
	control.linear.z = 0;
      
      ctrl_pub.publish(control);
      ptime = ctime;
      px = x;
      py = y;
      pz = z;
  }
  
  void setpointCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
      setx = msg->x;
      sety = msg->y;
      setz = msg->z;
      // Defining the safety cage
      if(setz < 0.5)
	setz = 0.5;
      if(setz > 1)
	setz = 1;
      if(setx > 1) setx = 1;
      if(setx < -1) setx = -1;
      if(sety > 1) sety = 1;
      if(sety < -1) sety = -1;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  droneController dr;
  ros::spin();
  return 0;
}