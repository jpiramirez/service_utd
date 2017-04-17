// Set-point controller for the rover
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016

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
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


using namespace std;
using namespace Eigen;


class roverController
{
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber point_sub;
  ros::Publisher viz_pub;
  double x, y, z, theta;
  ros::Time ptime, ctime;
  ros::Timer vtimer;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  nav_msgs::Odometry roverpose;
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
    ctrl_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    string odomtopic;
    nh_.param<std::string>("odometry_topic", odomtopic, "p3dx/base_pose_ground_truth");
    pose_sub = nh_.subscribe(odomtopic, 2, &roverController::Callback, this);
    viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("mapviz", 2);
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
    gsl_rng_set(RNG, time(NULL));

    um = new urbanmap();
    string mapname;
    nh_.param<std::string>("mapname", mapname, "test.yml");
    um->loadMap(mapname);

    vtimer = nh_.createTimer(ros::Duration(0.1), &roverController::visualize, this);

    isRoverOnMap = false;
    isRoverTraveling = false;
  }

  ~roverController()
  {
      control.linear.x = 0;
      control.angular.z = 0;
      ctrl_pub.publish(control);
      delete um;
  }

  void Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();

     // if(t < 0.02)
      //    return;
      roverpose = *msg;
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      z = msg->pose.pose.position.z;
      geometry_msgs::Quaternion q = msg->pose.pose.orientation;
      theta = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
      // theta = 2.0*asin(msg->pose.pose.orientation.z);

      float KPx = 0.2; //defaults 0.1 0.1 0.2 0.2 0.2 0.4
      float KPy = 0.4;
      float KPz = 0.8;

      float KDx = 0.8;
      float KDy = 0.8;
      float KDz = 1.2;

      float Ky = 2;

      double anglediff = 0.0;
      Point2d vg;
      std::pair<int, double> cnode;
      cnode = findClosestNode(Point2d(x, y));

      if(isRoverOnMap == false)
      {

          goal = um->coord[cnode.first];
          vg = goal-Point2d(x, y);
          if(norm(vg) < 1e-2)
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
      anglediff = cos(theta)*vg.x + sin(theta)*vg.y;
      anglediff /= sqrt(vg.x*vg.x + vg.y*vg.y);
      anglediff = acos(anglediff);
      // Getting the sign of vg cross vr
      anglediff = copysign(anglediff, cos(theta)*vg.y-sin(theta)*vg.x);
      if(isnan(anglediff))
        anglediff = 0.0;

      // while(fabs(anglediff) > M_PI)
      // {
      //     if(anglediff < -M_PI)
      //         anglediff += 2*M_PI;
      //     else
      //         anglediff -= 2*M_PI;
      // }
      // if(fabs(anglediff) > 5e-2)
      // {
          // control.linear.x = 0;
          control.angular.z = Ky*anglediff;
      // }
      // else
      // {
          // control.angular.z = 0;
          control.linear.x = KPx*norm(vg);
      // }

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

  void visualize(const ros::TimerEvent& te)
  {
    visualization_msgs::Marker marker;
    vector<visualization_msgs::Marker> markarr;
    visualization_msgs::MarkerArray markerarray;

    // Drawing the intersections

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    vector<geometry_msgs::Point> ptarray;
    for(int i=0; i < um->coord.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = um->coord[i].x;
      pt.y = um->coord[i].y;
      pt.z = 0;
      ptarray.push_back(pt);
    }
    marker.points = ptarray;

    markarr.push_back(marker);

    // Drawing the streets

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    ptarray.clear();
    for(int i=0; i < um->elist.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = um->coord[um->elist[i].x].x;
      pt.y = um->coord[um->elist[i].x].y;
      pt.z = 0;
      ptarray.push_back(pt);
      pt.x = um->coord[um->elist[i].y].x;
      pt.y = um->coord[um->elist[i].y].y;
      pt.z = 0;
      ptarray.push_back(pt);
    }
    marker.points = ptarray;

    markarr.push_back(marker);


    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 2;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = roverpose.pose.pose.position;
    marker.pose.position.z = 0.5;
    marker.pose.orientation = roverpose.pose.pose.orientation;
    marker.scale.x = 3;
    marker.scale.y = 3;
    marker.scale.z = 3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    ptarray.clear();
    marker.points = ptarray;
    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://service_utd/meshes/pioneer.dae";

    markarr.push_back(marker);

    markerarray.markers = markarr;

    viz_pub.publish(markerarray);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  roverController dr;
  ros::spin();
  return 0;
}
