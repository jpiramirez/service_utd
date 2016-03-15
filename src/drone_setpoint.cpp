// Set-point controller for the drone
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
#include <eigen3/Eigen/Geometry>
#include <math.h>

#define HORIZON 10

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
  float px, py, pz, ah, pah;
  float setx, sety, setz;
  float errx, erry, errz;
  float perrx, perry, perrz;
  float KPx, KPy, KPz, KDx, KDy, KDz, Ky;
  int cycles, burnin;
  bool launch;

public:
  droneController()
  {
    ctrl_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pose_sub = nh_.subscribe("ardrone/pose", 1, &droneController::Callback, this);
    point_sub = nh_.subscribe("ardrone/setpoint", 1, &droneController::setpointCallback, this);

    vector<double> gainlist;
    nh_.getParam("drone_setpoint/gains", gainlist);

    if(gainlist.size() == 0)
    {
        ROS_INFO_STREAM("Gains are not properly set.");
        ros::shutdown();
    }

    KPx = gainlist[0];
    KPy = gainlist[1];
    KPz = gainlist[2];
    KDx = gainlist[3];
    KDy = gainlist[4];
    KDz = gainlist[5];
    Ky  = gainlist[6];

    ROS_INFO_STREAM("Set point controller initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    px = 0;
    py = 0;
    pz = 0;
    setx = 0;
    sety = 0;
    setz = 1;
    errx = 0;
    erry = 0;
    errz = 0;
    ah = 0;
    perrx = 0;
    perry = 0;
    perrz = 0;
    pah = 0;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    while(ptime.toSec() == 0.0 || ctime.toSec() == 0.0)
    {
      ptime = ros::Time::now();
      ctime = ros::Time::now();
    }

    cycles = 0;
    burnin = 50;
    launch = false;
  }

  ~droneController()
  {
  }


  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

     if(t < 0.02 || launch == false)
         return;

      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;


      // This code assumes that the order in which the markers arrive is
      // front    back    left    right

      //odom = msg;


      float n = 1;

      double xsum = 0, ysum = 0, zsum = 0, asum = 0;

      double angle = 2.0*asin(msg->pose.orientation.z);
      geometry_msgs::Quaternion q = msg->pose.orientation;
      angle = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));

      float cx, cy;

      errx = (x - setx)/(n+1) + n*perrx/(n+1);
      erry = (y - sety)/(n+1) + n*perry/(n+1);
      errz = (z - setz)/(n+1) + n*perrz/(n+1);
      ah = angle/(n+1) + n*pah/(n+1);

      errx = x - setx;
      erry = y - sety;
      errz = z - setz;
      ah = angle;


      errx = setx - x;
      erry = sety - y;
      errz = setz - z;
      cx = KPx*errx + KDx*(errx-perrx)/t;
      cy = KPy*erry + KDy*(erry-perry)/t;
      control.linear.x = cos(ah)*cx + sin(ah)*cy;
      control.linear.y = -sin(ah)*cx + cos(ah)*cy;
      control.linear.z = KPz*errz + KDz*(errz-perrz)/t;

      control.angular.x = 0;
      control.angular.y = 0;
      control.angular.z = -Ky*ah;

      if(fabs(control.linear.x) > 2.0)
      {
        control.linear.x = copysign(2.0, control.linear.x);
      }
      if(fabs(control.linear.y) > 2.0)
      {
        control.linear.y = copysign(2.0, control.linear.y);
      }
      if(fabs(control.linear.z) > 2.0)
      {
        control.linear.z = copysign(2.0, control.linear.z);
      }
      if(fabs(control.angular.z) > 2.0)
      {
        control.angular.z = copysign(2.0, control.angular.z);
      }

      ctrl_pub.publish(control);
      ptime = ctime;

      px = errx;

      perrx = errx;
      perry = erry;
      perrz = errz;
      pah = ah;

  }

  void setpointCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
      launch = true;
      setx = msg->x;
      sety = msg->y;
      setz = msg->z;
      // Defining the safety cage
      if(setz < 0.5)
          setz = 0.5;
      if(setz > 25)
          setz = 25;
//      if(setx > 3) setx = 3;
//      if(setx < -3) setx = -3;
//      if(sety > 3) sety = 3;
//      if(sety < -3) sety = -3;
      ROS_INFO_STREAM("New setpoint: " << setx << " " << sety << " " << setz);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  droneController dr;
  ros::spin();
  return 0;
}
