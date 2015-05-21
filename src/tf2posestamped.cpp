#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2posestamped");

  ros::NodeHandle node;

  ros::Publisher pose =
    node.advertise<geometry_msgs::PoseStamped>("ardrone/pose", 10);

  tf::TransformListener listener;

  long int seq = 0;

  ros::Rate rate(50.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseStamped ps;
    ps.header.seq = seq;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    ps.pose.position.x = transform.getOrigin().x();
    ps.pose.position.y = transform.getOrigin().y();
    ps.pose.position.z = transform.getOrigin().z();
    ps.pose.orientation.w = transform.getRotation().w();
    ps.pose.orientation.x = transform.getRotation().x();
    ps.pose.orientation.y = transform.getRotation().y();
    ps.pose.orientation.z = transform.getRotation().z();
    pose.publish(ps);
    seq++;

    rate.sleep();
  }
  return 0;
};

