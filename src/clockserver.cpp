// Clock server

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class clockserver
{
  ros::NodeHandle nh_;
  ros::Publisher clockpub;
  ros::Time ctime;

  double tfactor, realfactor;

  ros::WallTimer timer;

public:
  clockserver()
  {

    nh_.param("timefactor", tfactor, 1.0);
    if(tfactor < 0.0)
      tfactor = 1.0;
    clockpub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 2);
//    point_sub = nh_.subscribe("ardrone/setpoint", 2, &clockserver::setpointCallback, this);
    ROS_INFO_STREAM("Clock server created");

    nh_.param("tickspersec", realfactor, 250.0);
    timer = nh_.createWallTimer(ros::WallDuration(1.0/realfactor), &clockserver::broadcastTime, this);

    ctime.sec = 0;
    ctime.nsec = 0;
  }

  ~clockserver()
  {
  }


  void broadcastTime(const ros::WallTimerEvent& te)
  {
      ros::Duration d(tfactor/realfactor);
      ctime = ctime + d;

      rosgraph_msgs::Clock clockmsg;
      clockmsg.clock.sec = ctime.sec;
      clockmsg.clock.nsec = ctime.nsec;

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
