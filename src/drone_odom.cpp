// Odometry for the drone
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <service_utd/Markers.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

class droneOdom
{
  ros::NodeHandle nh_;
  ros::Publisher odom_pub, tgt_pub;
  ros::Subscriber navdata_sub;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  geometry_msgs::PoseStamped odom, tgtodom;
  
public:
  droneOdom()
  {
    odom_pub = nh_.advertise<geometry_msgs::PoseStamped>("/ardrone/pose", 2);
    tgt_pub = nh_.advertise<geometry_msgs::PoseStamped>("/target/pose", 2);
    navdata_sub = nh_.subscribe("/vicon/markers", 2, &droneOdom::navdataCallback, this);
    ROS_INFO_STREAM("Zero point initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    odom.header.seq = 0;
    odom.header.frame_id = "map";
    tgtodom.header.seq = 0;
    tgtodom.header.frame_id = "map";
  }

  ~droneOdom()
  {
  }

  void navdataCallback(const service_utd::Markers::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();
      Vector3d front, back, left, right;
      Vector3d torig, tfront, tleft, tright;
      
      // This code assumes that the order in which the markers arrive is 
      // front    back    left    right
      bool dronevisible = true;

      for(int i=0; i < msg->markers.size(); i++)
      {
          if(msg->markers[i].subject_name.compare("ardrone2") == 0)
          {
              if(msg->markers[i].marker_name.compare("front") == 0)
                  front << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("back") == 0)
                  back << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("left") == 0)
                  left << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("right") == 0)
                  right << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].occluded == true)
                  dronevisible = false;
          }
          if(msg->markers[i].subject_name.compare("shoemaker") == 0)
          {
              if(msg->markers[i].marker_name.compare("front") == 0)
                  tfront << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("origin") == 0)
                  torig << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("left") == 0)
                  tleft << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
              if(msg->markers[i].marker_name.compare("right") == 0)
                  tright << 1e-3*msg->markers[i].translation.x, 1e-3*msg->markers[i].translation.y, \
                          1e-3*msg->markers[i].translation.z;
          }
      }
      
      Vector3d vec1 = front - back;
      Vector3d ivec(1, 0, 0);
      Vector3d flatv(vec1(0), vec1(1), 0);
      float cosyaw = ivec.dot(flatv)/flatv.norm();
      Vector3d crossv = ivec.cross(flatv);
      double yaw = copysign(acos(cosyaw), crossv(2));
      
      odom.header.seq++;
      ctime = ros::Time::now();
      odom.header.stamp = ctime;
      odom.pose.position.x = front[0];
      odom.pose.position.y = front[1];
      odom.pose.position.z = front[2];
      odom.pose.orientation.w = cos(yaw/2.0);
      odom.pose.orientation.x = 0.0;
      odom.pose.orientation.y = 0.0;
      odom.pose.orientation.z = sin(yaw/2.0);
      if(dronevisible)
        odom_pub.publish(odom);

      vec1 = tfront - torig;
      flatv << vec1(0), vec1(1), 0;
      cosyaw = ivec.dot(flatv)/flatv.norm();
      crossv = ivec.cross(flatv);
      yaw = copysign(acos(cosyaw), crossv(2));

      tgtodom.header.stamp = ctime;
      tgtodom.pose.position.x = torig[0];
      tgtodom.pose.position.y = torig[1];
      tgtodom.pose.position.z = torig[2];
      tgtodom.pose.orientation.w = cos(yaw/2.0);
      tgtodom.pose.orientation.z = sin(yaw/2.0);
      if(dronevisible)
        tgt_pub.publish(tgtodom);

      if(dronevisible)
        ptime = ctime;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_odom");
  droneOdom dr;
  ros::spin();
  return 0;
}
