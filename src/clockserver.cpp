// Clock server

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class clockserver
{
  ros::NodeHandle nh_;
  ros::Publisher clockpub;
  ros::WallTime ctime;

  double tfactor;

  ros::WallTimer timer;

public:
  clockserver()
  {

    nh_.param("timefactor", tfactor, 1.0);
    clockpub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 2);
//    point_sub = nh_.subscribe("ardrone/setpoint", 2, &clockserver::setpointCallback, this);
    ROS_INFO_STREAM("Clock server created");

    timer = nh_.createWallTimer(ros::WallDuration(0.004), &clockserver::broadcastTime, this);

  }

  ~clockserver()
  {
  }


  void broadcastTime(const ros::WallTimerEvent& te)
  {
      ctime = ros::WallTime::now();

      rosgraph_msgs::Clock clockmsg;
      clockmsg.clock.sec = tfactor*ctime.sec;
      clockmsg.clock.nsec = tfactor*ctime.nsec;

      clockpub.publish(clockmsg);
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clockserver");
  clockserver dr;
  ros::spin();
  return 0;
}
