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
  float errx, erry, errz;
  float perrx, perry, perrz;
  vector<float> xeh, yeh, zeh;
  
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
    errx = 0;
    erry = 0;
    errz = 0;
    perrx = 0;
    perry = 0;
    perrz = 0;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    for(int i=0; i < 5; i++)
    {
        xeh.push_back(0);
        yeh.push_back(0);
        zeh.push_back(0);
    }
  }

  ~droneController()
  {
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;


      // This code assumes that the order in which the markers arrive is 
      // front    back    left    right
      
      //odom = msg;
      float KPx = 0.1;
      float KPy = 0.1;
      float KPz = 0.1;
      
      float KDx = 0.5;
      float KDy = 0.5;
      float KDz = 0.5;
      
      float Ky = 5;
      

      errx = x - setx;
      erry = y - sety;
      errz = z - setz;

      double xsum = 0, ysum = 0, zsum = 0;
      for(int i=0; i < xeh.size()-1; i++)
      {
          xeh[i] = xeh[i+1];
          yeh[i] = yeh[i+1];
          zeh[i] = zeh[i+1];
          xsum += xeh[i];
          ysum += yeh[i];
          zsum += zeh[i];
      }
      xeh[xeh.size()] = (errx - perrx);
      yeh[yeh.size()] = (erry - perry);
      zeh[zeh.size()] = (errz - perrz);
      xsum += xeh[xeh.size()];
      ysum += yeh[yeh.size()];
      zsum += zeh[zeh.size()];
      
      double angle = 2.0*asin(msg->pose.orientation.z);
      
      //if(fabs(angle) < 1e-2)
      //angle = 0;
      if(fabs(angle) < 5e-1)
      {
          control.linear.x = -KPx*errx - KDx*(xsum/xeh.size())/t;
          control.linear.y = -KPy*erry - KDy*(ysum/yeh.size())/t;
          control.linear.z = -KPz*errz - KDz*(zsum/zeh.size())/t;
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
      //if(fabs(control.linear.x) > 0.05)
      //control.linear.x = copysign(0.05, control.linear.x);
      //if(fabs(control.linear.y) > 0.05)
      //control.linear.y = copysign(0.05, control.linear.y);
      //if(fabs(control.linear.z) > 0.05)
      //control.linear.z = copysign(0.05, control.linear.z);

      // Hover
      if(fabs(control.linear.x) < 1e-2)
          control.linear.x = 0;
      if(fabs(control.linear.y) < 1e-2)
          control.linear.y = 0;
      if(fabs(control.linear.z) < 1e-2)
          control.linear.z = 0;
      
      ctrl_pub.publish(control);
      ptime = ctime;

      px = xsum/xeh.size();
      py = ysum/yeh.size();
      pz = zsum/zeh.size();

      perrx = errx;
      perry = erry;
      perrz = errz;

  }
  
  void setpointCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
      setx = msg->x;
      sety = msg->y;
      setz = msg->z;
      // Defining the safety cage
      if(setz < 0.5)
          setz = 0.5;
      if(setz > 2)
          setz = 2;
      if(setx > 3) setx = 3;
      if(setx < -3) setx = -3;
      if(sety > 3) sety = 3;
      if(sety < -3) sety = -3;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  droneController dr;
  ros::spin();
  return 0;
}
