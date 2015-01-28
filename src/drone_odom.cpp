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
  ros::Publisher odom_pub;
  ros::Subscriber navdata_sub;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  
public:
  droneOdom()
  {
    odom_pub = nh_.advertise<geometry_msgs::PoseStamped>("/ardrone/pose", 2);
    navdata_sub = nh_.subscribe("/vicon/markers", 2, &droneOdom::navdataCallback, this);
    ROS_INFO_STREAM("Zero point initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    odom.header.seq = 0;
    odom.header.frame_id = "map";
  }

  ~droneOdom()
  {
  }

  void navdataCallback(const service_utd::Markers::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();
      
      // This code assumes that the order in which the markers arrive is 
      // front    back    left    right
      
      Vector3d front(1e-3*msg->markers[0].translation.x, 1e-3*msg->markers[0].translation.y, \
                     1e-3*msg->markers[0].translation.z);
      Vector3d back(1e-3*msg->markers[1].translation.x, 1e-3*msg->markers[1].translation.y, \
                     1e-3*msg->markers[1].translation.z);
      Vector3d left(1e-3*msg->markers[2].translation.x, 1e-3*msg->markers[2].translation.y, \
                     1e-3*msg->markers[2].translation.z);
      Vector3d right(1e-3*msg->markers[3].translation.x, 1e-3*msg->markers[3].translation.y, \
                     1e-3*msg->markers[3].translation.z);
      
      Vector3d vec1 = front - back;
      Vector3d ivec(1, 0, 0);
      Vector3d flatv(vec1(0), vec1(1), 0);
      float cosyaw = ivec.dot(flatv)/flatv.norm();
      Vector3d crossv = ivec.cross(flatv);
      double yaw = copysign(acos(cosyaw), crossv(2));
      cout << yaw << endl;
      cout << "www" << endl;
      
      odom.header.seq++;
      odom.header.stamp = ros::Time::now();
      odom.pose.position.x = msg->markers[0].translation.x*1e-3;
      odom.pose.position.y = msg->markers[0].translation.y*1e-3;
      odom.pose.position.z = msg->markers[0].translation.z*1e-3;
      odom.pose.orientation.w = cos(yaw/2.0);
      odom.pose.orientation.z = sin(yaw/2.0);
      odom_pub.publish(odom);
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
