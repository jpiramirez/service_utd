// Odometry for the drone
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <service_utd/Markers.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
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
  geometry_msgs::PoseStamped odom;
  nav_msgs::Odometry tgtodom;
  Eigen::Matrix3Xd dronepoints;
  int mcount;
  std::string dronename;
  bool brtarget;

public:
  droneOdom()
  {
    odom_pub = nh_.advertise<geometry_msgs::PoseStamped>("ardrone/pose", 2);
    tgt_pub = nh_.advertise<nav_msgs::Odometry>("/target/pose", 2);
    navdata_sub = nh_.subscribe("/vicon/markers", 2, &droneOdom::navdataCallback, this);

    nh_.param<std::string>("dronename", dronename, "ardrone2");
    nh_.param("broadcast_target", brtarget, true);
    ROS_INFO_STREAM("Zero point initialized.");
    ROS_INFO_STREAM("Providing odometry for drone: " << dronename);
    x = 0.0;
    y = 0.0;
    z = 0.0;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    odom.header.seq = 0;
    odom.header.frame_id = "map";
    tgtodom.header.seq = 0;
    tgtodom.header.frame_id = "map";
    dronepoints.resize(3, 4);
    dronepoints << 0, 1, 1, 1,
                   0, 1, 1, 1,
                   0, 1, 1, 1;
    mcount = 0;
  }

  ~droneOdom()
  {
  }

  // The following function is from Oleg Alexandrov at NASA
  // This function is in the public domain

  // Given two sets of 3D points, find the rotation + translation + scale
  // which best maps the first set to the second.
  // Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
  // The input 3D points are stored as columns.
  Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {
      // Default output
      Eigen::Affine3d A;
      A.linear() = Eigen::Matrix3d::Identity(3, 3);
      A.translation() = Eigen::Vector3d::Zero();
      if (in.cols() != out.cols())
          throw "Find3DAffineTransform(): input data mis-match";
      // First find the scale, by finding the ratio of sums of some distances,
      // then bring the datasets to the same scale.
      double dist_in = 0, dist_out = 0;
      for (int col = 0; col < in.cols()-1; col++) {
          dist_in += (in.col(col+1) - in.col(col)).norm();
          dist_out += (out.col(col+1) - out.col(col)).norm();
      }
      if (dist_in <= 0 || dist_out <= 0)
          return A;
      double scale = dist_out/dist_in;
      out /= scale;
      // Find the centroids then shift to the origin
      Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
      Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
      for (int col = 0; col < in.cols(); col++) {
          in_ctr += in.col(col);
          out_ctr += out.col(col);
      }
      in_ctr /= in.cols();
      out_ctr /= out.cols();
      for (int col = 0; col < in.cols(); col++) {
          in.col(col) -= in_ctr;
          out.col(col) -= out_ctr;
      }
      // SVD
      Eigen::MatrixXd Cov = in * out.transpose();
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
      // Find the rotation
      double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
      if (d > 0)
          d = 1.0;
      else
          d = -1.0;
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
      I(2, 2) = d;
      Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();
      // The final transform
      A.linear() = scale * R;
      A.translation() = scale*(out_ctr - R*in_ctr);
      return A;
  }

  void navdataCallback(const service_utd::Markers::ConstPtr& msg)
  {
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();
      Vector3d front, back, left, right;
      Vector3d torig, tfront, tleft, tright;
      Eigen::Matrix3Xd dronecurpos;

      // This code assumes that the order in which the markers arrive is
      // front    back    left    right
      bool dronevisible = true;

      for(int i=0; i < msg->markers.size(); i++)
      {
          if(msg->markers[i].subject_name.compare(dronename) == 0)
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
              if(msg->markers[i].marker_name.compare("back") == 0)
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

      Eigen::Quaterniond q;
      dronecurpos.resize(3,4);
      if(dronevisible)
      {
          dronecurpos << front(0), back(0), left(0), right(0),
                         front(1), back(1), left(1), right(1),
                         front(2), back(2), left(2), right(2);
          if(mcount < 10)
          {
              dronepoints << front(0), back(0), left(0), right(0),
                             front(1), back(1), left(1), right(1),
                             front(2), back(2), left(2), right(2);
              Eigen::Matrix3Xd originp = Eigen::Matrix3Xd::Zero(3, 4);
              //originp.resize(3, 4);
              // originp << front(0), front(0), front(0), front(0),
              //            front(1), front(1), front(1), front(1),
              //            front(2), front(2), front(2), front(2);
              dronepoints = dronepoints - originp;
          }

          Eigen::Affine3d T = Find3DAffineTransform(dronepoints, dronecurpos);
          q = T.rotation();
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
      odom.pose.position.x = front[0];
      odom.pose.position.y = front[1];
      odom.pose.position.z = front[2];
      odom.pose.orientation.w = q.w();
      odom.pose.orientation.x = q.x();
      odom.pose.orientation.y = q.y();
      odom.pose.orientation.z = q.z();
      if(dronevisible)
        odom_pub.publish(odom);

      vec1 = tfront - torig;
      flatv << vec1(0), vec1(1), 0;
      cosyaw = ivec.dot(flatv)/flatv.norm();
      crossv = ivec.cross(flatv);
      yaw = copysign(acos(cosyaw), crossv(2));

      tgtodom.header.stamp = ctime;
      tgtodom.header.frame_id = "map";
      tgtodom.pose.pose.position.x = torig[0];
      tgtodom.pose.pose.position.y = torig[1];
      tgtodom.pose.pose.position.z = torig[2];
      tgtodom.pose.pose.orientation.w = cos(yaw/2.0);
      tgtodom.pose.pose.orientation.z = sin(yaw/2.0);
      if(dronevisible && brtarget)
        tgt_pub.publish(tgtodom);

      if(dronevisible)
        ptime = ctime;

      mcount++;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_odom");
  droneOdom dr;
  ROS_INFO_STREAM("Starting to compute the ARDrone pose.");
  ros::spin();
  return 0;
}
