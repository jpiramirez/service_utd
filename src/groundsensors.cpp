// Unattended Ground Sensors (simulated)
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
#include <service_utd/SoftMeasure.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <math.h>
#include <iomanip>
#include <utility>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <visualization_msgs/Marker.h>


using namespace std;
using namespace Eigen;


class UGS
{
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub;
  ros::Publisher viz_pub;
  vector<ros::Publisher> sm_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber point_sub;
  vector<ros::Subscriber> UAVposes;
  double x, y, z, theta;
  ros::Time ptime, ctime;
  ros::Timer vtimer;
  ros::Duration d;
  geometry_msgs::PoseStamped odom;
  const gsl_rng_type *T;
  gsl_rng *RNG;
  double alpha, beta;
  vector<double> ugsx, ugsy;
  double detectrad, commradius;
  std::map<int, int> UAVid;
  vector<Vector3d> UAVst;
  vector<service_utd::SoftMeasure> smvec;

public:
  UGS()
  {
    string odomtopic;
    nh_.param<std::string>("odometry_topic", odomtopic, "p3dx/base_pose_ground_truth");
    pose_sub = nh_.subscribe(odomtopic, 2, &UGS::Callback, this);
    viz_pub = nh_.advertise<visualization_msgs::Marker>("ugs_visuals", 2);
    //    point_sub = nh_.subscribe("ardrone/setpoint", 2, &UGS::setpointCallback, this);
    ROS_INFO_STREAM("UGS initialized.");

    string ss;
    string topic;
    string prefix;
    nh_.param<std::string>("prefix", prefix, "/uav");
    /// List of UAVs that we transmit information to
    vector<int> connlist;
    nh_.getParam("UAVconnectivity", connlist);
    for(int i=0; i < connlist.size(); i++)
    {
      cout << "UAV " << connlist[i] << endl;
      ss = boost::lexical_cast<string>(connlist[i]);
      UAVid[connlist[i]] = i;
      topic = prefix + ss + "/ardrone/pose";
      UAVposes.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, \
        boost::bind(&UGS::UAVCallback, this, _1, connlist[i])));
        ROS_INFO_STREAM("Node " << nh_.getNamespace() << " subscribing to pose of " << ss);
        UAVst.push_back(Vector3d(0,0,0));
        topic = prefix + ss + "/ground_obs";
        sm_pub.push_back(nh_.advertise<service_utd::SoftMeasure>(topic, 1));
        ROS_INFO_STREAM("Enabling transmission to " << topic);
      }

      T = gsl_rng_mt19937;
      RNG = gsl_rng_alloc(T);
      gsl_rng_env_setup();


      nh_.getParam("ugs/x", ugsx);
      nh_.getParam("ugs/y", ugsy);
      nh_.param<double>("detection_radius", detectrad, 1);
      nh_.param<double>("comm_radius", commradius, 2);

      nh_.param<double>("alpha", alpha, 0.8);
      nh_.param<double>("beta", beta, 0.2);

      if(ugsx.size() != ugsy.size())
      {
        ROS_INFO_STREAM("The number of x and y coord for UGS does not match.");
        ros::shutdown();
      }

      service_utd::SoftMeasure sm;
      for(int i=0; i < ugsx.size(); i++)
      {
          sm.header.stamp = ros::Time::now();
          sm.ul.x = ugsx[i] + detectrad;
          sm.ul.y = ugsy[i] + detectrad;
          sm.br.x = ugsx[i] - detectrad;
          sm.br.y = ugsy[i] - detectrad;
          sm.alpha = alpha;
          sm.beta = beta;
          sm.measurement = false;
          ROS_INFO_STREAM("UGS " << i << " located at (" << ugsx[i] << "," << \
                          ugsy[i] << ")");
          smvec.push_back(sm);
      }

      vtimer = nh_.createTimer(ros::Duration(0.5), &UGS::visualCallback, this);

      ctime = ros::Time::now();
      ptime = ros::Time::now();
    }

    ~UGS()
    {
    }


    bool detectRover(int sensor)
    {
      bool detect = false;
      if(sqrt(pow(x-ugsx[sensor], 2.0) + pow(y-ugsy[sensor], 2.0)) < detectrad)
        detect = true;
      double die = gsl_rng_uniform(RNG);
      if(detect)
      {
        if(die < alpha)
        return true;
        else
        return false;
      }
      if(die < beta)
      return true;
      else
      return false;
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

      for(int i=0; i < smvec.size(); i++)
      {
        service_utd::SoftMeasure sm;
        if(detectRover(i))
        {
          sm.header.stamp = ros::Time::now();
          sm.ul.x = ugsx[i] + detectrad;
          sm.ul.y = ugsy[i] + detectrad;
          sm.br.x = ugsx[i] - detectrad;
          sm.br.y = ugsy[i] - detectrad;
          sm.alpha = alpha;
          sm.beta = beta;
          sm.measurement = true;
        }
        else
        {
          sm.header.stamp = ros::Time::now();
          sm.ul.x = ugsx[i] + detectrad;
          sm.ul.y = ugsy[i] + detectrad;
          sm.br.x = ugsx[i] - detectrad;
          sm.br.y = ugsy[i] - detectrad;
          sm.alpha = alpha;
          sm.beta = beta;
          sm.measurement = false;
        }
        smvec[i] = sm;
      }
    }

    // This callback uses some "advanced", or rather poorly documented functionality in ROS that allows
    // the user to add extra parameters.
    void UAVCallback(const ros::MessageEvent<geometry_msgs::PoseStamped const>& event, int detectedUAV)
    {
      ctime = ros::Time::now();
      d = ctime - ptime;
      if(d.toSec() < 0.25)
        return;

      const geometry_msgs::PoseStampedConstPtr& msg = event.getMessage();
      int callerId = UAVid[detectedUAV];



      UAVst[callerId] = Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

      for(int i=0; i < smvec.size(); i++)
      {
        if(sqrt(pow(ugsx[i]-UAVst[callerId][0],2.0) + pow(ugsy[i]-UAVst[callerId][1],2.0)) < commradius)
        {
          sm_pub[callerId].publish(smvec[i]);
          ROS_INFO_STREAM("UAV " << detectedUAV << " seen by UGS " << i);
        }

      }
      ptime = ctime;
    }

    void visualCallback(const ros::TimerEvent& te)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = nh_.getNamespace();
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = detectrad;
      marker.scale.y = detectrad;
      marker.scale.z = 0.1;
      marker.color.a = 0.5; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      vector<geometry_msgs::Point> ptarray;
      for(int i=0; i < ugsx.size(); i++)
      {
        geometry_msgs::Point pt;

        pt.x = ugsx[i];
        pt.y = ugsy[i];
        pt.z = 0.3;

        ptarray.push_back(pt);
      }
      marker.points = ptarray;

      viz_pub.publish(marker);
    }

  };

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "groundsensors");
    UGS dr;
    ros::spin();
    return 0;
  }
