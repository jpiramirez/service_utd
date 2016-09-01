// Information-based planner node
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <service_utd/ProbMap.h>
#include <service_utd/SoftMeasure.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "particlefilter.h"
#include "inh_particlefilter.hpp"
#include "urbanmap.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <utility>
#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/lexical_cast.hpp>
#include <service_utd/ParticleSet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#define VZERO 1e-100

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace boost;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class pathPlanner
{
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub;
  ros::Publisher map_pub;
  ros::Publisher searchmap_pub;
  ros::Publisher pclpub;
  ros::Publisher viz_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber object_sub;
  ros::Subscriber tgtpose_sub;
  ros::Subscriber softmeas_sub;
  vector<ros::Subscriber> otherUAVposes;
  vector<ros::Subscriber> pfListeners;
  ros::Publisher ownpf_pub;

  vector<Vec3d> otherUAVst, otherUAVpst, otherUAVvel;
  vector<bool> otherUAVvisible;
  vector<bool> collflag;
  vector<ros::Time> pestim;
  std::map<int, int> UAVid;
  std::map<int, int> UAVsense;
  std::map<int, service_utd::ParticleSet > pfset;

  geometry_msgs::Vector3 wp_msg;
  geometry_msgs::PoseStamped ownPose;
  geometry_msgs::Pose targetPosition;
  service_utd::ProbMap pm_msg, sm_msg;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  float px, py, pz;
  Vec3d ownVel;
  float setx, sety, setz;
//  particleFilter *pf;
  inhParticleFilter *pf;
  urbanmap *um;
  double maplen;
  float fv, fh;
  float av, ah;
  ros::Timer timer, timerpf;
  int seqnum;
  ofstream timestamps;
  float xp, yp;
  double Copt;
  vector<double> wpx, wpy, wpz;
  int wpcount;

  double alpha, beta;
  bool detected;

  bool targetFound;
  double confThresh;
  double detectConfidence;

  bool waypointNav;
  bool collided;
  bool isUAVinGraph;
  int cruiseAltitude;
  bool planMode, executeMode;

  double collisionRadius, detectionRadius;

  Point2d pt_from, pt_to;
  vector<int> flightPlan;

  const gsl_rng_type *T;
  gsl_rng *RNG;
  int myId;

  double baseAltitude;

  bool batchSimulation;
  ros::Time startT, endT;
  ros::Duration simlength;

  std::string prefix;

  int horizonparam;
  bool displayMap;

public:
  pathPlanner()
  {
    waypoint_pub = nh_.advertise<geometry_msgs::Vector3>("ardrone/setpoint", 1);
    map_pub = nh_.advertise<service_utd::ProbMap>("probmap", 1);
    searchmap_pub = nh_.advertise<service_utd::ProbMap>("searchmap", 1);
    ownpf_pub = nh_.advertise<service_utd::ParticleSet>("particle_set", 1);
    pose_sub = nh_.subscribe("ardrone/pose", 1, &pathPlanner::Callback, this);
    object_sub = nh_.subscribe("objectdetected", 1, &pathPlanner::detectionCallback, this);
    tgtpose_sub = nh_.subscribe("objectpose", 1, &pathPlanner::targetPositionCallback, this);
    softmeas_sub = nh_.subscribe("ground_obs", 1, &pathPlanner::softMeasurementCallback, this);
    //pclpub = nh_.advertise<PointCloud>("particles", 1);
    viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("visuals", 2);

    vector<int> connlist;
    nh_.getParam("UAVdetectability", connlist);
    vector<int> senslist;
    nh_.getParam("UAVconnectivity", senslist);
    string ss;
    string topic;
    nh_.param<std::string>("prefix", prefix, "/uav");
    nh_.param("plan_length", horizonparam, 5);
    for(int i=0; i < connlist.size(); i++)
    {
        cout << "UAV " << connlist[i] << endl;
        ss = boost::lexical_cast<string>(connlist[i]);
        UAVid[connlist[i]] = i;
        topic = prefix + ss + "/ardrone/pose";
        otherUAVposes.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, \
                                                                          boost::bind(&pathPlanner::otherUAVCallback, this, _1, connlist[i])));
        ROS_INFO_STREAM("Node " << nh_.getNamespace() << " subscribing to pose of " << ss);
        otherUAVst.push_back(Vec3d(0,0,0));
        otherUAVpst.push_back(Vec3d(0,0,0));
        otherUAVvel.push_back(Vec3d(0,0,0));
        otherUAVvisible.push_back(false);
        collflag.push_back(false);
        pestim.push_back(ros::Time::now());
    }

    for(int i=0; i < senslist.size(); i++)
    {
        cout << "UAV " << senslist[i] << endl;
        ss = boost::lexical_cast<string>(senslist[i]);
        UAVsense[senslist[i]] = i;
        topic = prefix + ss + "/particle_set";
        pfListeners.push_back(nh_.subscribe<service_utd::ParticleSet>(topic, 1, boost::bind(&pathPlanner::pfCallback, this, _1, senslist[i])));
        ROS_INFO_STREAM("Node " << nh_.getNamespace() << " subscribing to particles of " << ss);
        pfset[senslist[i]] = service_utd::ParticleSet();
    }

    cout << nh_.getNamespace() << " connected to:" << endl;
    for(int i=0; i < otherUAVposes.size(); i++)
    {
        cout << otherUAVposes[i].getTopic() << endl;
    }
    for(int i=0; i < pfset.size(); i++)
    {
        cout << pfListeners[i].getTopic() << endl;
    }


    ROS_INFO_STREAM("Set point controller initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    px = 0;
    py = 0;
    pz = 0;

    setx = 0;
    sety = 0;
    setz = 1.4;

    nh_.param<double>("alpha", alpha, 0.8);
    nh_.param<double>("beta", beta, 0.2);
    string mapname;
    nh_.param<std::string>("mapname", mapname, "test.yml");

    um = new urbanmap();
    um->loadMap(mapname);
    maplen = 0.0;
    for(int i=0; i < um->wayln.size(); i++)
      maplen += um->wayln[i];

    int npart;
    nh_.param<int>("num_particles", npart, 4000);

//    pf = new particleFilter(npart, alpha, beta, *um, 0.3);
    int memsize;
    nh_.param<int>("memsize", memsize, 100);
    pf = new inhParticleFilter(npart, alpha, beta, *um, 0.1, memsize);

    nh_.param<bool>("displaymap", displayMap, true);
    if(displayMap)
        namedWindow("pdf");

    double gamman = pow(alpha, alpha/(alpha-beta)) * pow(1-alpha , (1-alpha)/(alpha-beta));
    double gammad = pow(beta , beta/(alpha-beta)) * pow(1-beta , (1-beta)/(alpha-beta));

    double gamma = gamman/gammad;

    Copt = (gamma*(1-beta)-beta)/((alpha-beta)*(1+gamma));

    // These come from the bottom camera calibration for the ardrone2
    fv = 701.654116;
    fh = 700.490828;
    av = 2.0*atan(180.0/fv);
    ah = 2.0*atan(320.0/fh);
    xp = 0;
    yp = 0;
    seqnum = 0;

//    timer = nh_.createTimer(ros::Duration(0.2), &pathPlanner::computeWaypoint, this);
    timerpf = nh_.createTimer(ros::Duration(0.5), &pathPlanner::broadcastParticles, this);
    timer = nh_.createTimer(ros::Duration(0.2), &pathPlanner::publishWaypoint, this);

    ptime = ros::Time::now();
    ctime = ros::Time::now();

    targetFound = false;
    confThresh = 0.6;
    detectConfidence = 0;

    detected = false;

    nh_.param("waypointNav", waypointNav, false);

    nh_.param("batchsimulation", batchSimulation, false);
    if(batchSimulation)
    {
      startT.sec = 0;
      startT.nsec = 0;
      while(startT.toSec() == 0.0)
        startT = ros::Time::now();
      ROS_INFO_STREAM("Planner starts at " << startT.toSec());
    }

    if(waypointNav)
    {
        ROS_INFO_STREAM("Loading waypoint list");
        std::string xpts;
        nh_.param<std::string>("waypoints/x", xpts, "0 1 1 0");
        std::string ypts;
        nh_.param<std::string>("waypoints/y", ypts, "0 0 1 1");
        std::string zpts;
        nh_.param<std::string>("waypoints/z", zpts, "1 1 1 1");
        std::stringstream ssx(xpts);
        std::stringstream ssy(ypts);
        std::stringstream ssz(zpts);

        double td;
        while(!ssx.eof())
        {
          ssx >> td;
          wpx.push_back(td);
          ssy >> td;
          wpy.push_back(td);
          ssz >> td;
          wpz.push_back(td);
        }

        wpcount = 0;
        wp_msg.x = (double)wpx[0];
        wp_msg.y = (double)wpy[0];
        wp_msg.z = (double)wpz[0];
    }

    nh_.param<double>("collision_radius", collisionRadius, 2);
    nh_.param<double>("detection_radius", detectionRadius, 2);

    nh_.param<double>("base_altitude", baseAltitude, 2);

    collided = false;
    T = gsl_rng_mt19937;
    RNG = gsl_rng_alloc(T);
    gsl_rng_set(RNG, time(NULL));

    isUAVinGraph = false;

    ownVel = Vec3d(0, 0, 0);
    cruiseAltitude = 0;

    // This gets rid of the leading "//uav" to retrieve only the UAV number.
    // This requires the user to specify the namespace as "uav##"
    std::string myname = nh_.getNamespace();
    myname = myname.erase(0,5);
    myId = atoi(myname.c_str());

    executeMode = false;
    flightPlan.clear();
  }

  ~pathPlanner()
  {
      delete um;
      delete pf;
  }

  void softMeasurementCallback(const service_utd::SoftMeasure::ConstPtr& msg)
  {
      timesig t = msg->header.stamp;
      double sAlpha = msg->alpha;
      double sBeta = msg->beta;
      Point2d ul(msg->ul.x, msg->ul.y);
      Point2d br(msg->br.x, msg->br.y);
      ros::Duration d = ros::Time::now()-t;

      ROS_INFO_STREAM("Processing OOSM " << d.toSec() << " seconds old");
      pf->processOOSM(*um, ul, br, msg->measurement, t, sAlpha, sBeta);
  }

  void targetPositionCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
      targetPosition = *msg;
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      double xsum = 0, ysum = 0, zsum = 0;
      ctime = ros::Time::now();
      d = ctime - ptime;
      double t = d.toSec();
      //ROS_INFO_STREAM("Sample period: " << t);

      if(t < 0.1)
        return;
//      pf->predict(*um, t);
      pf->simplePredict(*um, 0.1);

      ownPose = *msg;

      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;

      // Sometimes TF or other sources of positional info cannot update fast
      // enough and they latch the latest measurement, resulting in an estimated
      // zero velocity. To avoid that, we estimate velocity using larger intervals
      // of 0.1 s or so.
      if(t > 0.1)
      {
          ownVel = Vec3d((x-px)/t, (y-py)/t, (z-pz)/t);
          px = x, py = y, pz = z;
          ptime = ctime;
      }

      if(waypointNav)
      {
          double dist = 0;
          dist += (x-wpx[wpcount])*(x-wpx[wpcount]);
          dist += (y-wpy[wpcount])*(y-wpy[wpcount]);
          dist += (z-wpz[wpcount])*(z-wpz[wpcount]);
          dist = sqrt(dist);

          if(dist < 0.1)
          {
              wpcount++;
              wpcount = wpcount % wpx.size();
              ROS_INFO_STREAM("Heading towards waypoint " << wpcount);
              wp_msg.x = wpx[wpcount];
              wp_msg.y = wpy[wpcount];
              wp_msg.z = wpz[wpcount];
              waypoint_pub.publish(wp_msg);
          }
      }

  }

  void detectionCallback(const std_msgs::Bool::ConstPtr& msg)
  {

      detected = msg->data;

      xp = 2.0*z*tan(fabs(av/2.0));
      yp = 2.0*z*tan(fabs(ah/2.0));

      // The camera is offset 2" from the drone center in the x-axis
      float x1 = x - 2.0*0.0254 - xp/2.0;
      float x2 = x - 2.0*0.0254 + xp/2.0;
      float y1 = y - yp/2.0;
      float y2 = y + yp/2.0;

//      pf->update(*um, Point2d(x2,y2), Point2d(x1,y1), detected);
      pf->simpleUpdate(*um, Point2d(x2,y2), Point2d(x1,y1), detected);

      double wsum = 0.0;
      Point2d pt;
      Point2d ul = Point2d(x2, y2);
      Point2d br = Point2d(x1, y1);
      if(detected)
      {
          for(int i=0; i < pf->N; i++)
          {
              pt = um->ep2coord((int)pf->pp[i][0], pf->pp[i][1]);
              if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                  wsum += pf->w[i];

          }
      }

      if(wsum > this->confThresh)
      {
          this->targetFound = true;
          ROS_INFO_STREAM("Target found by UAV " << this->myId);
          if(batchSimulation)
          {
              ROS_INFO_STREAM("Shutting down the node (batch simulations in progress)");
              endT = ros::Time::now();
              ROS_INFO_STREAM("Planner ends at " << endT.toSec());
              simlength = endT - startT;
              time_t tt;
              time(&tt);
              string stt = boost::lexical_cast<string>(tt);
              stt = stt + ".txt";
              ofstream fs;
              fs.open(stt.c_str());
              fs << simlength.toSec() << endl;
              fs.close();
              ROS_INFO_STREAM("Acquisition time recorded as " << stt);
              ros::shutdown();
          }
      }
      else
      {
          if(targetFound)
              isUAVinGraph = false;
          targetFound = false;

          // if(batchSimulation)
          // {
          //   endT = ros::Time::now();
          //   simlength = endT - startT;
          //   if(simlength.toSec() > 1000)
          //   {
          //     ROS_INFO_STREAM("Declaring target lost.");
          //     ros::shutdown();
          //   }
          // }
      }

      if(targetFound)
      {
          Point2d tgtpos(targetPosition.position.x, targetPosition.position.y);
//          Vec3f tgtpos = pf->meanEstim(*um);
          pf->particleFilter::update(*um, Point2d(x,y)+tgtpos, 1);
      }

      // Visualizing the pf

//      Mat visimagesc = Mat::zeros(500, 500, CV_8UC3);
//      um->drawMap(visimagesc);
//      pf->drawParticles(*um, visimagesc);
//      Point uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x2,y2));
//      Point uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x1,y1));
//      rectangle(visimagesc, uv1, uv2, CV_RGB(255, 0, 0), 3);
//      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(wp_msg.x, wp_msg.y));
//      circle(visimagesc, uv1, 10, CV_RGB(0,255,0));
//      imshow("pdf", visimagesc);
//      waitKey(3);



      seqnum++;
  }

  // This callback uses some "advanced", or rather poorly documented functionality in ROS that allows
  // the user to add extra parameters.
  void otherUAVCallback(const ros::MessageEvent<geometry_msgs::PoseStamped const>& event, int detectedUAV)
  {
        const geometry_msgs::PoseStampedConstPtr& msg = event.getMessage();
        int callerId = UAVid[detectedUAV];

//        if(myId == callerId)
//            return;

        ros::Time ctime = ros::Time::now();
        d = ctime - pestim[callerId];

        double t = d.toSec();


        double distanceToUAV = 0;
        distanceToUAV += pow(msg->pose.position.x-x, 2.0);
        distanceToUAV += pow(msg->pose.position.y-y, 2.0);
        distanceToUAV += pow(msg->pose.position.z-z, 2.0);
        distanceToUAV = sqrt(distanceToUAV);

        if(distanceToUAV < collisionRadius)
        {
          ROS_WARN_STREAM("UAV " << myId << " in imminent collision");
          collflag[callerId] = true;
        }
        else
          collflag[callerId] = false;

        // if(distanceToUAV < 1.5)
        // {
        //   ROS_FATAL_STREAM("UAV " << myId << " has collided.");
        //   ros::shutdown();
        // }

        if(distanceToUAV > detectionRadius)
        {
          otherUAVvisible[callerId] = false;
          return;
        }

        otherUAVvisible[callerId] = true;

        otherUAVst[callerId] = Vec3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        if(t > 0.1)
        {
            otherUAVvel[callerId] = (otherUAVst[callerId]-otherUAVpst[callerId])/t;

            otherUAVpst[callerId] = otherUAVst[callerId];
            pestim[callerId] = ctime;
        }

  }

  void broadcastParticles(const ros::TimerEvent& te)
  {
      bool broadcast = false;

//      for(int i=0; i < otherUAVst.size(); i++)
//      {
//          double distanceToUAV = 0;
//          distanceToUAV += pow(otherUAVst[i][0]-x, 2.0);
//          distanceToUAV += pow(otherUAVst[i][1]-y, 2.0);
//          distanceToUAV += pow(otherUAVst[i][2]-z, 2.0);
//          distanceToUAV = sqrt(distanceToUAV);
//          if(distanceToUAV < detectionRadius)
//              broadcast = true;
//      }

      broadcast = isUAVinGraph || targetFound;

      if(broadcast)
      {
          service_utd::ParticleSet pfmsg;
          pfmsg.header.stamp = ros::Time::now();
          pfmsg.header.frame_id = "world";
          pfmsg.N = this->pf->N;
          for(int i=0; i < pfmsg.N; i++)
          {
              geometry_msgs::Vector3 v;
              std_msgs::Float64 f64;
              v.x = pf->pp[i][0];
              v.y = pf->pp[i][1];
              v.z = pf->pp[i][2];
              pfmsg.data.push_back(v);
              f64.data = pf->w[i];
              if(isnan(pf->w[i]))
                  f64.data = 0.0;
              pfmsg.w.push_back(f64);
          }
          ownpf_pub.publish(pfmsg);
      }

      ROS_INFO_STREAM(nh_.getNamespace() << " broadcasting");
  }

  void pfCallback(const ros::MessageEvent<service_utd::ParticleSet const>& event, int detectedUAV)
  {
      const service_utd::ParticleSetConstPtr& msg = event.getMessage();

      int callerId = UAVsense[detectedUAV];
      pfset[callerId] = *msg;

      bool receiving = false;

//      for(int i=0; i < otherUAVst.size(); i++)
//      {
//          double distanceToUAV = 0;
//          distanceToUAV += pow(otherUAVst[i][0]-x, 2.0);
//          distanceToUAV += pow(otherUAVst[i][1]-y, 2.0);
//          distanceToUAV += pow(otherUAVst[i][2]-z, 2.0);
//          distanceToUAV = sqrt(distanceToUAV);
//          if(distanceToUAV < detectionRadius)
//              broadcast = true;
//      }

      ROS_DEBUG_STREAM("UAV " << myId << " checking if it receives " << detectedUAV << " with ID " << callerId);

      int collId = UAVid[detectedUAV];
      double distanceToUAV = 0;
      distanceToUAV += pow(otherUAVst[collId][0]-x, 2.0);
      distanceToUAV += pow(otherUAVst[collId][1]-y, 2.0);
      distanceToUAV += pow(otherUAVst[collId][2]-z, 2.0);
      distanceToUAV = sqrt(distanceToUAV);
      if(distanceToUAV < detectionRadius)
          receiving = true;


      if(!receiving || msg->N != pf->N)
          return;

      ROS_INFO_STREAM(nh_.getNamespace() << " fusing particles from " << detectedUAV << " with ID " << callerId);


        /*****************************
         * Computing per-edge statistics and using them to update the particle set
         * **************************/

        vector<double> Emean(um->elist.size(), 0.0);
        vector<double> Evar(um->elist.size(), 0.0);
        vector<double> Efactor(um->elist.size(), 0.0);

        // Channel filter
        vector<double> LEfactor(um->elist.size(), 0.0);
        vector<int> ppe(um->elist.size(), 0);
        double aff = 0.0;
        for(int i=0; i < pf->N; i++)
        {
            geometry_msgs::Vector3 v = msg->data[i];
            double w = msg->w[i].data;
            Emean[(int)v.x] += w*v.y;
            Evar[(int)v.x] += w*v.y*v.y;
            Efactor[(int)v.x] += w;
            LEfactor[(int)pf->pp[i][0]] += pf->w[i];
            ppe[(int)pf->pp[i][0]] += 1;
        }

        double lsum = 0.0, esum = 0.0;
        for(int i=0; i < Evar.size(); i++)
        {
          Efactor[i] *= um->wayln[i];
          LEfactor[i] *= um->wayln[i];
          lsum += LEfactor[i];
          esum += Efactor[i];
        }
        for(int i=0; i < Evar.size(); i++)
           aff += sqrt(Efactor[i]+LEfactor[i]);

        vector<double> fusion(um->elist.size(), 0.0);
        for(int i=0; i < Evar.size(); i++)
        {
          fusion[i] = Efactor[i]+LEfactor[i]+2.0*sqrt(Efactor[i]*LEfactor[i]);
          fusion[i] /= 2.0 + 2.0*aff;
        }

        double wsum = 0.0;
        for(int i=0; i < pf->N; i++)
        {
            int enumber = (int)pf->pp[i][0];

            if(ppe[enumber] > 0)
              pf->w[i] = fusion[enumber]/(ppe[enumber]*um->wayln[enumber]);
            else
              pf->w[i] = 0.0;
            wsum += pf->w[i];
        }
        if(wsum < 1e-100){
            pf->reset(*um);
            return;
        }
        for(int i=0; i < pf->N; i++)
            pf->w[i] /= wsum;

        pf->phist.putElement(pf->pp, 0);
        pf->whist.putElement(pf->w, 0);
        pf->tstamp.putElement(ros::Time::now(), 0);
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

      ROS_DEBUG_STREAM(pt << " " << closestE << " " << dmin << " " << um->elist[closestE]);
      ROS_DEBUG_STREAM("from " << um->coord[um->elist[closestE].x] << " to " << um->coord[um->elist[closestE].y]);

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
      // Point2d vl, vc;
      // vc = pt - lineStart;
      // vl = lineEnd - lineStart;
      // if( (vc.x*vl.y-vc.y*vl.x) <= radius*norm(vl) )
      // {
      //     if(norm(vc) < radius)
      //         coll = true;
      //     if(norm(vl-vc) < radius)
      //         coll = true;
      //     if(!coll && vc.dot(vl) >= 0.0 && vc.dot(vl) <= vl.dot(vl) )
      //         coll = true;
      // }
      Point2d vl, vc, ve;
      vc = pt - lineStart;
      vl = lineEnd - lineStart;
      ve = pt - lineEnd;
      if(norm(vc) < radius)
        return true;
      if(norm(ve) < radius)
        return true;
      double len = norm(vl);
      Point2d vln = Point2d(vl.x/len, vl.y/len);
      double proj = vc.dot(vln);
      Point2d pvec = proj*vln;
      Point2d rej = vc - pvec;

      if(proj > 0.0 && proj < len && norm(rej) <= radius)
        return true;


      return coll;
  }

  Vec3f targetMLE()
  {
      Vec3f mle(0,0,0);
      for(int i=0; i < pf->N; i++)
      {
          mle = mle + pf->w[i]*pf->pp[i];
      }
      mle[0] = round(mle[0]);
      return mle;
  }

  void publishWaypoint(const ros::TimerEvent& te)
  {
    bool coll = false;
    bool climb = false;
    Point2d pt;

    // if(executeMode == true)
    // {
    //   if(flightPlan.empty())
    //   {
    //     executeMode = false;
    //   }
    // }
    // else if(isUAVinGraph)
    // {
    //   planPath();
    //   executeMode = true;
    // }

    if(flightPlan.empty() && isUAVinGraph)
    {
      planPath();
    }



    for(int i=0; i < otherUAVst.size(); i++)
    {
        if(norm(Point2d(x,y) - Point2d(otherUAVst[i][0], otherUAVst[i][1])) < 5.0)
        {
            coll = true;
            if(norm(Point2d(x,y)) < norm(Point2d(otherUAVst[i][0], otherUAVst[i][1])))
                climb = true;
        }
    }

    if(cruiseAltitude == 0)
    {
        if(coll)
        {
            if(climb)
                cruiseAltitude = 1;
            else
                cruiseAltitude = -1;
        }
    }
    else if(cruiseAltitude == 1)
    {
        if(!coll)
            cruiseAltitude = 0;
    }
        else
    {
        if(!coll)
            cruiseAltitude = 0;
    }

//      if(coll)
//      {
//          pt = Point2d(x,y);
//          ROS_INFO_STREAM("Collision imminent, stopping.");
//      }

    int idx;
    double d;


    if(!(isUAVinGraph || targetFound))
    {
        tie(idx, d) = findClosestNode(Point2d(x,y));
        if(norm(Point2d(x,y) - um->coord[idx]) > 0.1)
        {
            wp_msg.x = um->coord[idx].x;
            wp_msg.y = um->coord[idx].y;
            wp_msg.z = baseAltitude;
            cout << "STATE: Approaching the road network" << endl;
            pt_to = um->coord[idx];
        }
        else
            isUAVinGraph = true;
    }
    else
    {

        if(targetFound)
        {
            wp_msg.x = x+targetPosition.position.x;
            wp_msg.y = y+targetPosition.position.y;
            Vec3f tgtpos = pf->meanEstim(*um);
            Point2d tgtestim = um->ep2coord((int)tgtpos[0], tgtpos[1]);
            wp_msg.x = tgtestim.x;
            wp_msg.y = tgtestim.y;
            cout << "STATE: Target detected, following it." << endl;
        }
        else
        {
            if(norm(Point2d(x,y) - um->coord[flightPlan.back()]) < 0.1)
            {
                flightPlan.pop_back();
                if(flightPlan.empty())
                  return;
            }
            pt = um->coord[flightPlan.back()];

            wp_msg.x = pt.x;
            wp_msg.y = pt.y;
            pt_from = pt_to;
            pt_to = pt;
            cout << "STATE: Traveling to the next waypoint" << endl;

        }
    }


    switch(cruiseAltitude)
    {
      case(1):
      if(batchSimulation)
        wp_msg.z = baseAltitude + myId - 1;
      break;
      case(-1):
      if(batchSimulation)
        wp_msg.z = baseAltitude + myId - 1;
      break;
      default:
      wp_msg.z = baseAltitude;
      break;
    }

    if(coll)
      cout << "UAV " << myId << " climbing to " << wp_msg.z << endl;

    waypoint_pub.publish(wp_msg);



    //RVIZ visualization

    visualization_msgs::Marker marker;
    vector<visualization_msgs::Marker> markarr;
    visualization_msgs::MarkerArray markerarray;

    // Drawing the particle filter
    double meanw = 0.0;
    for(int i=0; i < pf->N; i++)
    {
        if(!isnan(pf->w[i]))
          meanw += pf->w[i];
    }
    meanw /= (double)pf->N;

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
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    vector<geometry_msgs::Point> ptarray;
    for(int i=0; i < pf->N; i++)
    {
      geometry_msgs::Point pt;
      if(pf->w[i] >= meanw)
      // if(weight[(int)pf->pp[i][0]] >= 1000)
      {
        Point2d p = um->ep2coord((int)pf->pp[i][0], pf->pp[i][1]);
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.2;
        ptarray.push_back(pt);
      }
    }
    marker.points = ptarray;

    markarr.push_back(marker);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    ptarray.clear();
    for(int i=0; i < pf->N; i++)
    {
      geometry_msgs::Point pt;
      if(pf->w[i] < meanw)
      // if(weight[(int)pf->pp[i][0]] < 1000)
      {
        Point2d p = um->ep2coord((int)pf->pp[i][0], pf->pp[i][1]);
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.2;
        ptarray.push_back(pt);
      }
    }
    marker.points = ptarray;

    markarr.push_back(marker);

    // Drawing the quad
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 2;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ownPose.pose.position.x;
    marker.pose.position.y = ownPose.pose.position.y;
    marker.pose.position.z = ownPose.pose.position.z;
    marker.pose.orientation.x = ownPose.pose.orientation.x;
    marker.pose.orientation.y = ownPose.pose.orientation.y;
    marker.pose.orientation.z = ownPose.pose.orientation.z;
    marker.pose.orientation.w = ownPose.pose.orientation.w;
    marker.scale.x = 3.0;
    marker.scale.y = 3.0;
    marker.scale.z = 3.0;
    marker.color.a = 0.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://service_utd/meshes/quadrotor_base.dae";

    markarr.push_back(marker);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 3;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ownPose.pose.position.x;
    marker.pose.position.y = ownPose.pose.position.y;
    marker.pose.position.z = ownPose.pose.position.z+0.5;
    marker.pose.orientation.x = ownPose.pose.orientation.x;
    marker.pose.orientation.y = ownPose.pose.orientation.y;
    marker.pose.orientation.z = ownPose.pose.orientation.z;
    marker.pose.orientation.w = ownPose.pose.orientation.w;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.text = nh_.getNamespace();
    markarr.push_back(marker);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 4;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ownPose.pose.position.x;
    marker.pose.position.y = ownPose.pose.position.y;
    marker.pose.position.z = ownPose.pose.position.z;
    marker.pose.orientation.x = ownPose.pose.orientation.x;
    marker.pose.orientation.y = ownPose.pose.orientation.y;
    marker.pose.orientation.z = ownPose.pose.orientation.z;
    marker.pose.orientation.w = ownPose.pose.orientation.w;
    marker.scale.x = collisionRadius;
    marker.scale.y = collisionRadius;
    marker.scale.z = collisionRadius;
    marker.color.a = 0.25; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text = nh_.getNamespace();
    markarr.push_back(marker);

    markerarray.markers = markarr;
    viz_pub.publish(markerarray);

//      um->coord.pop_back();
  }

  void planPath()
  {

      typedef adjacency_list<vecS, vecS, undirectedS, \
              property<vertex_index_t, int>, property<edge_weight_t, double> > Graph;
      typedef property_map<Graph, edge_weight_t>::type WeightMap;
      typedef graph_traits<Graph >::vertex_descriptor vertex_descriptor;
      typedef property_map<Graph, vertex_index_t>::type IndexMap;
      typedef std::pair<int, int> E;

      bool onEdge = false, onNode = false;

      vector<double> wt;

      vector<int> chList;
      vector<int> pList;
      vector<double> aList;

      double planningHoriz = 20.0;

      int idx;
      double d;

      tie(idx, d) = findClosestEdge(Point2d(x,y));
      if(d < 0.1)
          onEdge = true;
      tie(idx, d) = findClosestNode(Point2d(x,y));
      if(d < 0.1)
          onNode = true;


      ROS_DEBUG_STREAM("The edge closest to the UAV is " << idx << ",");
      ROS_DEBUG_STREAM(" from vertex " << um->elist[idx].x << " to " << um->elist[idx].y);
//      pList.push_back(um->elist[findClosestEdge(Point2d(x,y))].x);
      pList.push_back(idx);


      ROS_DEBUG_STREAM("Nav starts from node " << pList[0] << " at " << um->coord[pList[0]]);


      Graph::edge_descriptor e;
      bool inserted;


      Point2d pt;
//      Vec3f mle = this->targetMLE();
      Vec3f mle = pf->meanEstim(*um);
      Point2d mlePos = um->ep2coord((int)mle[0], mle[1]);

      double proj = 0;
      int vcount = 1;
      E *Edge = new E[um->elist.size()];
      double *weight = new double[um->elist.size()];

      // Compute the time it will take the UAV to reach the current destination node

      ROS_DEBUG_STREAM(nh_.getNamespace() << " velocity is " << norm(ownVel));
      double timeToDest = norm(Point2d(wp_msg.x-x, wp_msg.y-y))/norm(ownVel);
      ROS_DEBUG_STREAM("Time to dest " << timeToDest);
      vector<double> estimRad;
      // Lookahead horizon
      int numTravEdges = horizonparam;

      for(int k=0; k < um->elist.size(); k++)
      {

          double sc = 0.0;
          for(int j=0; j < pf->N; j++)
          {
              bool isPinE = um->elist[(int)pf->pp[j][0]].x == um->elist[k].x && \
                      um->elist[(int)pf->pp[j][0]].y == um->elist[k].y;
              isPinE |= um->elist[(int)pf->pp[j][0]].y == um->elist[k].x && \
                      um->elist[(int)pf->pp[j][0]].x == um->elist[k].y;

              if(isPinE)
              {
                  sc += pf->w[j];
//                  sc += 1.0;
              }
          }

          // Find if the route will result in a collision
          double coll = 0.0;
          Point2d ls, le;
          ls = um->coord[um->elist[k].x];
          le = um->coord[um->elist[k].y];
          for(int j=0; j < otherUAVst.size(); j++)
          {
              if(otherUAVvisible[j] == false)
                continue;
              Point2d uavcoord = Point2d(otherUAVst[j][0], otherUAVst[j][1]);

              double radtemp = 2*timeToDest*norm(otherUAVvel[j]);
              // if(radtemp < collisionRadius || norm(ownVel) < 1e-6)
                radtemp = collisionRadius;
              estimRad.push_back(radtemp);
              // if(norm(ownVel) < 1e-6)
                  // estimRad[j] = collisionRadius;
//              cout << "estimRad " << estimRad[j] << endl;

              if(lineSegmentCollision(uavcoord, estimRad[j], ls, le))
                 coll += 1000.0;

          }

          Point2d midpt = Point2d(x,y)-0.5*Point2d(ls.x+le.x,ls.y+le.y);
          double dweight = fabs(midpt.x*midpt.x + midpt.y*midpt.y);
//          weight[k] = um->wayln[k]/(1.0+sc) + coll;
          // if(sc < 1.0)
            // weight[k] = 1.0-sc + coll;

        /// Cost Function
         sc = sc*um->wayln[k]/maplen;
        //  weight[k] = fabs(1.0-sc) + coll;
            weight[k] = fabs(sc - 0.5/(double)(horizonparam)) + coll;
          // else
            // weight[k] = coll;

          // tie(idx, d) = findClosestEdge(Point2d(x,y));
          // if(k == idx)
          //   weight[k] += 1000;
          wt.push_back(weight[k]);

          Edge[k] = E(um->elist[k].x, um->elist[k].y);

          vcount++;

      }

//      Graph G(Edge, Edge + sizeof(Edge) / sizeof(E), weight, um->coord.size());
      Graph G(um->coord.size());
      for(int i=0; i < um->elist.size(); i++)
      {
          add_edge(Edge[i].first, Edge[i].second, wt[i], G);
      }

      vertex_descriptor searchStart;
      tie(idx, d) = findClosestNode(Point2d(x,y));
      searchStart = idx;

      IndexMap index = get(vertex_index, G);

      ROS_DEBUG_STREAM("edges(g) = ");
      graph_traits<Graph>::edge_iterator ei, ei_end;

      vector<double> dist(num_vertices(G));
      vector<vertex_descriptor> p(num_vertices(G));
      ROS_DEBUG_STREAM("The graph has " << num_vertices(G) << " vertices.");
      vertex_descriptor s = *(vertices(G).first);
      s = pList[0];
      s = searchStart;
      dijkstra_shortest_paths(G, s, predecessor_map(&p[0]).distance_map(&dist[0]));

      int minidx = *(vertices(G).first);
      double mindist = 1e300;

      graph_traits<Graph>::vertex_iterator vi, vend;
      vector<vertex_descriptor> validNodes;

      for(tie(vi, vend) = vertices(G); vi!=vend; ++vi)
      {
          int npred = 0;
          int sv = index[*vi];
          while(sv != s)
          {
              sv = p[sv];
              npred++;
          }
          if(npred == numTravEdges)
          {
              validNodes.push_back(index[*vi]);
          }
      }

//      for(tie(vi, vend) = vertices(G); vi!=vend; ++vi)
      for(int i=0; i < validNodes.size(); i++)
      {
//          cout << "Distance to vertex " << index(*vi) << " is " << d[*vi] << endl;
//          if(dist[*vi] < mindist && index(*vi) != s && norm(Point2d(x,y)-um->coord[index[*vi]]) > planningHoriz)
          vertex_descriptor vd = validNodes[i];
          if(dist[index[vd]] < mindist && index[vd] != s)
          {
              minidx = index[vd];
              mindist = dist[vd];
          }

      }

      // If there are no particles within 2 edges of the starting node, find the shortest path
      // to the MLE
      // if(fabs(mindist-(double)numTravEdges) < 1e-80)
      // {
      //     tie(idx, d) = findClosestNode(mlePos);
      //     minidx = idx;
      // }

      ROS_DEBUG_STREAM("Minimum distance is " << mindist << " to node " << minidx << " at " << um->coord[minidx]);

      int vidx = minidx;
//      tie(vidx, d) = findClosestNode(mlePos);
      vector<int> idxpath;
      idxpath.push_back(vidx);
      while(vidx != s)
      {
          idxpath.push_back(p[vidx]);
          vidx = p[vidx];
      }

      for(vector<int>::iterator it=idxpath.begin(); it!=idxpath.end(); ++it)
      {
          cout << *it << ' ';
      }

      pt = um->coord[idxpath[idxpath.size()-2]];

      flightPlan = idxpath;
      // flightPlan.pop_back();

      if(displayMap)
      {
        xp = 2.0*z*tan(fabs(av/2.0));
        yp = 2.0*z*tan(fabs(ah/2.0));

        // The camera is offset 2" from the drone center in the x-axis
        float x1 = x - 2.0*0.0254 - xp/2.0;
        float x2 = x - 2.0*0.0254 + xp/2.0;
        float y1 = y - yp/2.0;
        float y2 = y + yp/2.0;

        // Mat visimagesc = Mat::zeros(500, 500, CV_8UC3);
        Mat visimagesc(500, 500, CV_8UC3, Scalar(255,255,255));
        um->drawMap(visimagesc);
        pf->drawParticles(*um, visimagesc);
        Point uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x2,y2));
        Point uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x1,y1));
        rectangle(visimagesc, uv1, uv2, CV_RGB(255, 0, 0), 3);
        cv::putText(visimagesc, nh_.getNamespace().c_str(), uv2, FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0));
        uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(wp_msg.x, wp_msg.y));
        circle(visimagesc, uv1, 10, CV_RGB(0,255,0));

        //      um->coord.push_back(Point2d(x,y));

        for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
        {
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[index[source(*ei, G)]]);
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[index[target(*ei, G)]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(0,0,0), 1);
          cv::circle(visimagesc, uv2, 3, CV_RGB(255,255,255));
        }

        uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[idxpath[0]]);
        cv::circle(visimagesc, uv1, 8, CV_RGB(0,0,0));
        for(int i=1; i < idxpath.size(); i++)
        {
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[idxpath[i]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(0,255,0), 2);
          uv1 = uv2;
        }

        double rad;
        for(int j=0; j < otherUAVst.size(); j++)
        {
          if(otherUAVvisible[j] == false)
          continue;
          pt = Point2d(otherUAVst[j][0], otherUAVst[j][1]);
          // uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, pt);
          // // int rad = estimRad[j]*visimagesc.rows/(um->ne.x - um->sw.x);
          // // cv::circle(visimagesc, uv1, rad, CV_RGB(100,0,0), 2);
          rad = detectionRadius*visimagesc.rows/(um->ne.x - um->sw.x);
          cv::circle(visimagesc, uv1, rad, CV_RGB(100,100,100), 2);
          rad = collisionRadius*visimagesc.rows/(um->ne.x - um->sw.x);
          cv::circle(visimagesc, uv1, rad, CV_RGB(255,0,0), 2);
        }


        pt = um->ep2coord((int)mle[0], mle[1]);
        uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, pt);
        circle(visimagesc, uv1, 5, CV_RGB(0,255,255), -1);

        imshow("pdf", visimagesc);
        waitKey(3);
      }

      delete[] weight;
      delete[] Edge;
  }

  std::pair<int, double> findNearestCoord(vector<Point2d> v, Point2d pt)
  {
      int nc = 0;
      double dist = norm(v[0]-pt);
      for(int i=1; i < v.size(); i++)
      {
          if(norm(v[i]-pt) < dist)
          {
              dist = norm(v[i]-pt);
              nc = i;
          }
      }
      return std::pair<int, double>(nc, dist);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanner");
  pathPlanner dr;
  ros::spin();
  return 0;
}
