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
  float xeh[HORIZON], yeh[HORIZON], zeh[HORIZON], anh[HORIZON];

  
public:
  droneController()
  {
    ctrl_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    pose_sub = nh_.subscribe("ardrone/pose", 2, &droneController::Callback2, this);
    point_sub = nh_.subscribe("ardrone/setpoint", 2, &droneController::setpointCallback, this);
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

    for(int i=0; i < HORIZON; i++)
    {
        xeh[i] = 0;
        yeh[i] = 0;
        zeh[i] = 0;
        anh[i] = 0;
    }
  }

  ~droneController()
  {
  }

//  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//  {
//      ctime = ros::Time::now();
//      d = ctime - ptime;
//      double t = d.toSec();

//      x = msg->pose.position.x;
//      y = msg->pose.position.y;
//      z = msg->pose.position.z;


//      // This code assumes that the order in which the markers arrive is
//      // front    back    left    right
      
//      //odom = msg;
//      float KPx = 0.1;
//      float KPy = 0.1;
//      float KPz = 0.2;
      
//      float KDx = 0.01;
//      float KDy = 0.01;
//      float KDz = 0.02;
      
//      float Ky = 5;
      

//      errx = x - setx;
//      erry = y - sety;
//      errz = z - setz;

//      double xsum = 0, ysum = 0, zsum = 0;
//      for(int i=0; i < xeh.size()-1; i++)
//      {
//          xeh[i] = xeh[i+1];
//          yeh[i] = yeh[i+1];
//          zeh[i] = zeh[i+1];
//          xsum += xeh[i];
//          ysum += yeh[i];
//          zsum += zeh[i];
//      }
//      xeh[xeh.size()] = (errx - perrx);
//      yeh[yeh.size()] = (erry - perry);
//      zeh[zeh.size()] = (errz - perrz);
//      xsum += xeh[xeh.size()];
//      ysum += yeh[yeh.size()];
//      zsum += zeh[zeh.size()];
      
//      double angle = 2.0*asin(msg->pose.orientation.z);
      
//      //if(fabs(angle) < 1e-2)
//      //angle = 0;
//      if(fabs(angle) < 5e-2)
//      {
//          control.linear.x = -KPx*errx - KDx*(xsum/xeh.size())/t;
//          control.linear.y = -KPy*erry - KDy*(ysum/yeh.size())/t;
//          control.linear.z = -KPz*errz - KDz*(zsum/zeh.size())/t;
//      }
//      else
//      {
//          control.linear.x = 0;
//          control.linear.y = 0;
//          control.linear.z = 0;
//      }
//      control.angular.x = 0;
//      control.angular.y = 0;
//      control.angular.z = -Ky*angle;
      
      
//      // Saturation
//      //if(fabs(control.linear.x) > 0.05)
//      //control.linear.x = copysign(0.05, control.linear.x);
//      //if(fabs(control.linear.y) > 0.05)
//      //control.linear.y = copysign(0.05, control.linear.y);
//      //if(fabs(control.linear.z) > 0.05)
//      //control.linear.z = copysign(0.05, control.linear.z);

//      // Hover
//      if(fabs(control.linear.x) < 1e-2)
//          control.linear.x = 0;
//      if(fabs(control.linear.y) < 1e-2)
//          control.linear.y = 0;
//      if(fabs(control.linear.z) < 1e-2)
//          control.linear.z = 0;
      
//      ctrl_pub.publish(control);
//      ptime = ctime;

//      px = xsum/xeh.size();
//      py = ysum/yeh.size();
//      pz = zsum/zeh.size();

//      perrx = errx;
//      perry = erry;
//      perrz = errz;

//  }

  void Callback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

     // if(t < 0.02)
      //    return;

      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;


      // This code assumes that the order in which the markers arrive is
      // front    back    left    right

      //odom = msg;
      float KPx = 0.2; //defaults 0.1 0.1 0.2 0.2 0.2 0.4
      float KPy = 0.2;
      float KPz = 0.4;

      float KDx = 0.4;
      float KDy = 0.4;
      float KDz = 0.6;

      float Ky = 2;

      float n = 1;

      double xsum = 0, ysum = 0, zsum = 0, asum = 0;

      double angle = 2.0*asin(msg->pose.orientation.z);
      geometry_msgs::Quaternion q = msg->pose.orientation;
      angle = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
     // cout << "Yaw: " << angle << endl;

      for(int i=0; i < HORIZON-1; i++)
      {
          xeh[i] = xeh[i+1];
          yeh[i] = yeh[i+1];
          zeh[i] = zeh[i+1];
          anh[i] = anh[i+1];
          xsum += xeh[i];
          ysum += yeh[i];
          zsum += zeh[i];
          asum += anh[i];

      }

      xeh[HORIZON-1] = (x);
      yeh[HORIZON-1] = (y);
      zeh[HORIZON-1] = (z);
      anh[HORIZON-1] = angle;
      xsum += xeh[HORIZON-1];
      ysum += yeh[HORIZON-1];
      zsum += zeh[HORIZON-1];
      asum += anh[HORIZON-1];

//      for(int i=0; i < HORIZON; i++)
//          cout << xeh[i] << " ";
//      cout << endl;

      //errx = xsum/float(HORIZON) - setx;
      //erry = ysum/float(HORIZON) - sety;
      //errz = zsum/float(HORIZON) - setz;
      //angle = asum/float(HORIZON);

      float cx, cy;
      //if(fabs(angle) < 1e-2)
      //angle = 0;

      errx = (x - setx)/(n+1) + n*perrx/(n+1);
      erry = (y - sety)/(n+1) + n*perry/(n+1);
      errz = (z - setz)/(n+1) + n*perrz/(n+1);
      ah = angle/(n+1) + n*pah/(n+1);

      errx = x - setx;
      erry = y - sety;
      errz = z - setz;
      ah = angle;

      //cx = -KPx*errx - KDx*(errx-perrx)/t;
      //cy = -KPy*erry - KDy*(erry-perry)/t;
      //control.linear.z = -KPz*errz - KDz*(errz-perrz)/t;

      errx = setx - x;
      erry = sety - y;
      errz = setz - z;
      cx = KPx*errx + KDx*(errx-perrx)/t;
      cy = KPy*erry + KDy*(erry-perry)/t;
      control.linear.x = cos(ah)*cx + sin(ah)*cy;
      control.linear.y = -sin(ah)*cx + cos(ah)*cy;
      control.linear.z = KPz*errz + KDz*(errz-perrz)/t;

//      control.linear.x = errx;
//      control.linear.y = erry;
//      control.linear.z = errz;

      control.angular.x = 0;
      control.angular.y = 0;
      control.angular.z = -Ky*ah;


      ctrl_pub.publish(control);
      ptime = ctime;

      px = errx;
//      px = xsum/xeh.size();
//      py = ysum/yeh.size();
//      pz = zsum/zeh.size();

      perrx = errx;
      perry = erry;
      perrz = errz;
      pah = ah;

  }
  
  void setpointCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
      setx = msg->x;
      sety = msg->y;
      setz = msg->z;
      // Defining the safety cage
      if(setz < 0.5)
          setz = 0.5;
      if(setz > 2.5)
          setz = 2.5;
      if(setx > 3) setx = 3;
      if(setx < -3) setx = -3;
      if(sety > 3) sety = 3;
      if(sety < -3) sety = -3;
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
