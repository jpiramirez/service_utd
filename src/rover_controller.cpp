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
#include "urbanmap.hpp"


using namespace std;
using namespace Eigen;


class roverController
{
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber point_sub;
  double x, y, z, theta;
  ros::Time ptime, ctime;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  geometry_msgs::Twist control;
  float px, py, pz, ah, pah;
  float setx, sety, setz;
  float errx, erry, errz;
  float perrx, perry, perrz;
  urbanmap *um;
  bool isRoverOnMap, isRoverTraveling;
  Point2d goal;
  const gsl_rng_type *T;
  gsl_rng *RNG;

public:
  roverController()
  {
    ctrl_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    string odomtopic;
    nh_.param<std::string>("odometry_topic", odomtopic, "p3dx/base_pose_ground_truth");
    pose_sub = nh_.subscribe(odomtopic, 2, &roverController::Callback, this);
//    point_sub = nh_.subscribe("ardrone/setpoint", 2, &roverController::setpointCallback, this);
    ROS_INFO_STREAM("Rover set point controller initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    theta = 0;
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

    T = gsl_rng_mt19937;
    RNG = gsl_rng_alloc(T);
    gsl_rng_env_setup();

    um = new urbanmap();
    string mapname;
    nh_.param<std::string>("mapname", mapname, "test.yml");
    um->loadMap(mapname);

    isRoverOnMap = false;
    isRoverTraveling = false;
  }

  ~roverController()
  {
      delete um;
  }

  void Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

     // if(t < 0.02)
      //    return;

      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      z = msg->pose.pose.position.z;
      theta = 2.0*asin(msg->pose.pose.orientation.z);

      float KPx = 0.4; //defaults 0.1 0.1 0.2 0.2 0.2 0.4
      float KPy = 0.4;
      float KPz = 0.8;

      float KDx = 0.8;
      float KDy = 0.8;
      float KDz = 1.2;

      float Ky = 1;

      double anglediff = 0.0;
      Point2d vg;
      std::pair<int, double> cnode;
      cnode = findClosestNode(Point2d(x, y));

      if(isRoverOnMap == false)
      {

          goal = um->coord[cnode.first];
          vg = goal-Point2d(x, y);
          if(norm(vg) < 1e-3)
              isRoverOnMap = true;
      }
      else
      {
          if(isRoverTraveling)
          {
              vg = goal-Point2d(x, y);
              if(norm(vg) < 1e-1)
                  isRoverTraveling = false;
          }
          else
          {
              int nnode = gsl_rng_uniform_int(RNG, um->conlist[cnode.first].size());
              nnode = um->conlist[cnode.first][nnode];
              goal = um->coord[nnode];
              isRoverTraveling = true;
          }

      }


      vg = goal-Point2d(x, y);
      anglediff = atan2(vg.y, vg.x) - theta;
      while(fabs(anglediff) > 2*M_PI)
      {
          if(anglediff < 0)
              anglediff += 2*M_PI;
          else
              anglediff -= 2*M_PI;
      }
      if(fabs(anglediff) > 1e-1)
      {
          control.linear.x = 0;
          control.angular.z = Ky*anglediff;
      }
      else
      {
          control.angular.z = 0;
          control.linear.x = KPx*norm(vg);
      }

      if(control.linear.x > 0.2)
          control.linear.x = 0.2;
      ctrl_pub.publish(control);
  }


  double linePointDistance(Point2d pt, Point2d lineStart, Point2d lineEnd)
  {
      double d = 0.0;
      d = fabs((lineEnd.x-lineStart.x)*(lineStart.y-pt.y)-(lineStart.x-pt.x)*(lineEnd.y-lineStart.y));
      d /= norm(lineEnd-lineStart);
      return d;
  }

  std::pair<int, double> findClosestEdge(Point2d pt)
  {
      int closestE = 0;
      int ls, le;
      ls = um->elist[0].x;
      le = um->elist[0].y;
      double dmin = 1e300;
      double d;
      double da, db;
      for(int i=0; i < um->elist.size(); i++)
      {
          ls = um->elist[i].x;
          le = um->elist[i].y;
          Point2d a = um->coord[ls];
          Point2d b = um->coord[le];
          Point2d vap = pt - a;
          Point2d vbp = pt - b;
          Point2d vab = b - a;
          Point2d vba = a - b;
          if(vap.dot(vab) > 0 && vbp.dot(vba) > 0)
              d = linePointDistance(pt, a, b);
          else
          {
              da = norm(pt - a);
              db = norm(pt - b);
              if(da < db)
                  d = da;
              else
                  d = db;
          }
          if(d < dmin)
          {
              closestE = i;
              dmin = d;
          }
      }

      cout << pt << " " << closestE << " " << dmin << " " << um->elist[closestE] << endl;
      cout << "from " << um->coord[um->elist[closestE].x] << " to " << um->coord[um->elist[closestE].y] << endl;

      return std::pair<int, double>(closestE, dmin);
  }

  std::pair<int, double> findClosestNode(Point2d pt)
  {
      int closestN = 0;
      double dmin = norm(pt - um->coord[0]);
      double d;
      for(int i=1; i < um->coord.size(); i++)
      {
          d = norm(pt - um->coord[i]);
          if(d < dmin)
          {
              closestN = i;
              dmin = d;
          }
      }

      return std::pair<int, double>(closestN, dmin);
  }

  bool lineSegmentCollision(Point2d pt, double radius, Point2d lineStart, Point2d lineEnd)
  {
      bool coll = false;
      Point2d vl, vc;
      vc = pt - lineStart;
      vl = lineEnd - lineStart;
      if( (vc.x*vl.y-vc.y*vl.x) <= radius*norm(vl) )
      {
          if(norm(vc) < radius)
              coll = true;
          if(norm(vl-vc) < radius)
              coll = true;
          if(!coll && vc.dot(vl) >= 0.0 && vc.dot(vl) <= vl.dot(vl) )
              coll = true;
      }


      return coll;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  roverController dr;
  ros::spin();
  return 0;
}
