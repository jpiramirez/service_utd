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
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "particlefilter.h"
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
  ros::Subscriber pose_sub;
  ros::Subscriber object_sub;
  ros::Subscriber tgtpose_sub;
  vector<ros::Subscriber> otherUAVposes;
  vector<ros::Subscriber> pfListeners;
  ros::Publisher ownpf_pub;

  vector<Vec3d> otherUAVst, otherUAVpst, otherUAVvel;
  vector<ros::Time> pestim;
  std::map<int, int> UAVid;
  std::map<int, service_utd::ParticleSet > pfset;

  geometry_msgs::Vector3 wp_msg;
  geometry_msgs::Pose targetPosition;
  service_utd::ProbMap pm_msg, sm_msg;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  float px, py, pz;
  Vec3d ownVel;
  float setx, sety, setz;
  particleFilter *pf;
  urbanmap *um;
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

  double collisionRadius, detectionRadius;

  const gsl_rng_type *T;
  gsl_rng *RNG;
  int myId;

  double baseAltitude;

public:
  pathPlanner()
  {
    waypoint_pub = nh_.advertise<geometry_msgs::Vector3>("ardrone/setpoint", 2);
    map_pub = nh_.advertise<service_utd::ProbMap>("probmap", 2);
    searchmap_pub = nh_.advertise<service_utd::ProbMap>("searchmap", 2);
    ownpf_pub = nh_.advertise<service_utd::ParticleSet>("particle_set", 1);
    pose_sub = nh_.subscribe("ardrone/pose", 2, &pathPlanner::Callback, this);
    object_sub = nh_.subscribe("objectdetected", 2, &pathPlanner::detectionCallback, this);
    tgtpose_sub = nh_.subscribe("objectpose", 1, &pathPlanner::targetPositionCallback, this);
    pclpub = nh_.advertise<PointCloud>("particles", 1);

    vector<int> connlist;
    nh_.getParam("UAVconnectivity", connlist);
    string ss;
    string topic;
    string prefix = "/uav";
    for(int i=0; i < connlist.size(); i++)
    {
        cout << "UAV " << connlist[i] << endl;
        ss = boost::lexical_cast<string>(connlist[i]);
        UAVid[connlist[i]] = i;
        topic = prefix + ss + "/ardrone/pose";
        otherUAVposes.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, \
                                                                          boost::bind(&pathPlanner::otherUAVCallback, this, _1, connlist[i])));
        topic = prefix + ss + "/particle_set";
        pfListeners.push_back(nh_.subscribe<service_utd::ParticleSet>(topic, 1, boost::bind(&pathPlanner::pfCallback, this, _1, connlist[i])));
        ROS_INFO_STREAM("Node " << nh_.getNamespace() << " subscribing to " << ss);
        otherUAVst.push_back(Vec3d(0,0,0));
        otherUAVpst.push_back(Vec3d(0,0,0));
        otherUAVvel.push_back(Vec3d(0,0,0));
        pestim.push_back(ros::Time::now());
        pfset[connlist[i]] = service_utd::ParticleSet();
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
    ptime = ros::Time::now();
    ctime = ros::Time::now();

    gsl_rng_default_seed = ctime.nsec;

    nh_.param<double>("alpha", alpha, 0.8);
    nh_.param<double>("beta", beta, 0.2);
    string mapname;
    nh_.param<std::string>("mapname", mapname, "test.yml");

    um = new urbanmap();
    um->loadMap(mapname);

    int npart;
    nh_.param<int>("num_particles", npart, 4000);

    pf = new particleFilter(npart, alpha, beta, *um, 0.3);

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
    timerpf = nh_.createTimer(ros::Duration(1.0), &pathPlanner::broadcastParticles, this);
    timer = nh_.createTimer(ros::Duration(0.2), &pathPlanner::pathPlanOverMap, this);
    namedWindow("pdf");

    ptime = ros::Time::now();
    ctime = ros::Time::now();

    targetFound = false;
    confThresh = 0.6;
    detectConfidence = 0;

    detected = false;

    nh_.param("waypointNav", waypointNav, false);

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
    gsl_rng_env_setup();

    isUAVinGraph = false;

    ownVel = Vec3d(0, 0, 0);
    cruiseAltitude = 0;

    // This gets rid of the leading "//uav" to retrieve only the UAV number. This requires the user to
    // specify the namespace as "uav##"
    std::string myname = nh_.getNamespace();
    myname = myname.erase(0,5);
    myId = atoi(myname.c_str());
  }

  ~pathPlanner()
  {
      delete um;
      delete pf;
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

      pf->predict(*um, t);

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

      pf->update(*um, Point2d(x2,y2), Point2d(x1,y1), detected);

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
      }
      else
      {
          if(targetFound)
              isUAVinGraph = false;
          targetFound = false;
      }

      if(targetFound)
      {
          Point2d tgtpos(targetPosition.position.x, targetPosition.position.y);
          pf->update(*um, Point2d(x,y)+tgtpos, 0.5);
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

        if(myId == detectedUAV)
            return;

        ros::Time ctime = ros::Time::now();
        d = ctime - pestim[callerId];

        double t = d.toSec();


        double distanceToUAV = 0;
        distanceToUAV += pow(msg->pose.position.x-x, 2.0);
        distanceToUAV += pow(msg->pose.position.y-y, 2.0);
        distanceToUAV += pow(msg->pose.position.z-z, 2.0);
        distanceToUAV = sqrt(distanceToUAV);


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

      for(int i=0; i < otherUAVst.size(); i++)
      {
          double distanceToUAV = 0;
          distanceToUAV += pow(otherUAVst[i][0]-x, 2.0);
          distanceToUAV += pow(otherUAVst[i][1]-y, 2.0);
          distanceToUAV += pow(otherUAVst[i][2]-z, 2.0);
          distanceToUAV = sqrt(distanceToUAV);
          if(distanceToUAV < detectionRadius)
              broadcast = true;
      }

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
              pfmsg.w.push_back(f64);
          }
          ownpf_pub.publish(pfmsg);
      }
  }

  void pfCallback(const ros::MessageEvent<service_utd::ParticleSet const>& event, int detectedUAV)
  {
      const service_utd::ParticleSetConstPtr& msg = event.getMessage();

      int callerId = UAVid[detectedUAV];
      pfset[callerId] = *msg;

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

      double distanceToUAV = 0;
      distanceToUAV += pow(otherUAVst[callerId][0]-x, 2.0);
      distanceToUAV += pow(otherUAVst[callerId][1]-y, 2.0);
      distanceToUAV += pow(otherUAVst[callerId][2]-z, 2.0);
      distanceToUAV = sqrt(distanceToUAV);
      if(distanceToUAV < detectionRadius)
          broadcast = true;


      if(!broadcast || msg->N != pf->N || isUAVinGraph == false)
          return;

      ROS_INFO_STREAM(nh_.getNamespace() << " fusing PFs");

//      double wsum = 0.0;
//      double fuseRad = 5;
//      for(int i=0; i < pf->N; i++)
//      {
//          Point2d ploc = um->ep2coord((int)pf->pp[i][0], pf->pp[i][1]);
//          double wfac = 0.0;
//          for(int j=0; j < msg->N; j++)
//          {
//              if((int)pf->pp[i][0] != (int)msg->data[j].x)
//                  continue;
//              Point2d ploc2 = um->ep2coord((int)msg->data[j].x, msg->data[j].y);
//              if(norm(ploc-ploc2) < fuseRad)
//              {
//                  wfac += msg->w[j].data;
//              }
//          }
//          pf->w[i] *= wfac;
//          wsum += pf->w[i];
//      }

//      for(int i=0; i < pf->N; i++)
//        pf->w[i] /= wsum;

      // Will attempt to fuse the particle sets and resample from them

      vector<double> u, wc;
      vector<int> ind(pf->N);
      vector<Vec3f> fusedpp;
      vector<double> fusedw;
      double totalW = 0;

      for(int i=0; i < pf->N; i++)
      {
          fusedpp.push_back(pf->pp[i]);
          Vec3f particle;
          particle[0] = msg->data[i].x;
          particle[1] = msg->data[i].y;
          particle[2] = msg->data[i].z;
          fusedpp.push_back(particle);
          fusedw.push_back(pf->w[i]);
          totalW += pf->w[i];
          double weight = msg->w[i].data;
          fusedw.push_back(weight);
          totalW += weight;
      }

      // We now have a fused particle set with 2N points.
      // Let's resample it, but only N times.

      for(int i=0; i < fusedpp.size(); i++)
      {
          fusedw[i] /= totalW;

          if(i == 0)
              wc.push_back(fusedw[0]);
          else
              wc.push_back(wc[i-1] + fusedw[i]);

      }


//      int k = 0;
//      for(int i=0; i < pf->N; i++)
//      {
//          u.push_back((gsl_rng_uniform(RNG) + i)/(double)pf->N);
//          while(wc[k] < u[i])
//              k++;
//          ind[i] = k;
//      }
      for(int i=0; i < pf->N; i++)
          ind[i] = gsl_rng_uniform_int(RNG, 2*pf->N);

      vector<Vec3f> npp;
      for(int i=0; i < pf->N; i++)
          npp.push_back(Vec3f(0,0,0));
      for(int i=0; i < pf->N; i++)
      {
          npp[i] = fusedpp[ind[i]];
          pf->w[i] = 1.0/(double)pf->N;
      }

      pf->pp = npp;
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

  void pathPlanOverMap(const ros::TimerEvent& te)
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



      PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "world";
        msg->height = msg->width = 1;
        msg->width = pf->N;
        for(int i=0; i < pf->N; i++)
        {
            Point2d ptmp = um->ep2coord((int)pf->pp[i][0], pf->pp[i][1]);
            msg->points.push_back (pcl::PointXYZ(ptmp.x, ptmp.y, 0.0));
        }
        msg->header.stamp = ros::Time::now().toNSec();
        pclpub.publish (msg);

      tie(idx, d) = findClosestEdge(Point2d(x,y));
      if(d < 0.1)
          onEdge = true;
      tie(idx, d) = findClosestNode(Point2d(x,y));
      if(d < 0.1)
          onNode = true;


      cout << "The edge closest to the UAV is " << idx << ",";
      cout << " from vertex " << um->elist[idx].x << " to " << um->elist[idx].y << endl;
//      pList.push_back(um->elist[findClosestEdge(Point2d(x,y))].x);
      pList.push_back(idx);


      cout << "Nav starts from node " << pList[0] << " at " << um->coord[pList[0]] << endl;


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

      cout << nh_.getNamespace() << " velocity is " << norm(ownVel);
      double timeToDest = norm(Point2d(wp_msg.x-x, wp_msg.y-y))/norm(ownVel);
      cout << "Time to dest " << timeToDest << endl;
      vector<double> estimRad;

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
              Point2d uavcoord = Point2d(otherUAVst[j][0], otherUAVst[j][1]);


              estimRad.push_back(timeToDest*norm(otherUAVvel[j]));
              if(norm(ownVel) < 1e-6)
                  estimRad[j] = collisionRadius;
//              cout << "estimRad " << estimRad[j] << endl;

              if(lineSegmentCollision(uavcoord, estimRad[j], ls, le))
                 coll += 1000.0;
          }

//          weight[k] = um->wayln[k]/(1.0+sc) + coll;
          if(sc < 1.0)
            weight[k] = 1-sc + coll;
          else
            weight[k] = coll;
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
//      if(norm(um->coord[idx] - Point2d(x,y)) > 0.5)
//      {
//          int clEdge;
//          tie(clEdge, idx) = findClosestEdge(Point2d(x,y));

//          Point2d ls, le1, le2;
//          ls = Point2d(x,y);
//          le1 = um->coord[um->elist[clEdge].x];
//          le2 = um->coord[um->elist[clEdge].y];
//          double sc1 = 0, sc2 = 0;

//          // Checking if any particles fit in the new edges to we can compute
//          // proper edge weights.
//          for(int j=0; j < pf->N; j++)
//          {

//              Point2d partLoc = um->ep2coord((int)pf->pp[j][0], pf->pp[j][1]);
//              bool isPinE = lineSegmentCollision(partLoc, 0.5, ls, le1);
//              if(isPinE)
//                    sc1 += pf->w[j];
//              isPinE = lineSegmentCollision(partLoc, 0.5, ls, le2);
//              if(isPinE)
//                    sc2 += pf->w[j];

//          }
//          add_edge(um->elist[clEdge].x, um->coord.size(), 1-sc1, G);
//          tie(e, inserted) = add_edge(um->elist[clEdge].y, um->coord.size(), 1-sc2, G);
//          searchStart = target(e, G);
//      }
//      else
          searchStart = idx;

      IndexMap index = get(vertex_index, G);

      std::cout << "edges(g) = ";
      graph_traits<Graph>::edge_iterator ei, ei_end;
//      for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
//          std::cout << "(" << index[source(*ei, G)] \
//                    << "," << index[target(*ei, G)] << ") ";

      vector<double> dist(num_vertices(G));
      vector<vertex_descriptor> p(num_vertices(G));
      cout << "The graph has " << num_vertices(G) << " vertices." << endl;
      vertex_descriptor s = *(vertices(G).first);
      s = pList[0];
      s = searchStart;
      dijkstra_shortest_paths(G, s, predecessor_map(&p[0]).distance_map(&dist[0]));

      int minidx = *(vertices(G).first);
      double mindist = 1e300;

      graph_traits<Graph>::vertex_iterator vi, vend;
      vector<vertex_descriptor> validNodes;

      // Lookahead horizon
      int numTravEdges = 2;

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
      if(fabs(mindist-(double)numTravEdges) < 1e-6)
      {
          tie(idx, d) = findClosestNode(mlePos);
          minidx = idx;
      }

      cout << "Minimum distance is " << mindist << " to node " << minidx << " at " << um->coord[minidx] << endl;

      int vidx = minidx;
//      tie(vidx, d) = findClosestNode(mlePos);
      vector<int> idxpath;
      idxpath.push_back(vidx);
      while(vidx != s)
      {
          idxpath.push_back(p[vidx]);
          vidx = p[vidx];
      }

      cout << "Da path" << endl;
      for(vector<int>::iterator it=idxpath.begin(); it!=idxpath.end(); ++it)
      {
          cout << *it << ' ';
      }

      pt = um->coord[idxpath[idxpath.size()-2]];
      bool coll = false;
      bool climb = false;
      for(int i=0; i < otherUAVst.size(); i++)
      {
          if(norm(Point2d(x,y) - Point2d(otherUAVst[i][0], otherUAVst[i][1])) < collisionRadius)
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

      tie(idx, d) = findClosestNode(Point2d(x,y));

      if(!isUAVinGraph)
      {
          tie(idx, d) = findClosestNode(Point2d(x,y));
          if(norm(Point2d(x,y) - um->coord[idx]) > 0.1 && !targetFound)
          {
              wp_msg.x = um->coord[idx].x;
              wp_msg.y = um->coord[idx].y;
              wp_msg.z = baseAltitude;
              cout << "STATE: Approaching the road network" << endl;
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
              cout << "STATE: Target detected, following it." << endl;
          }
          else if(norm(Point2d(x,y) - um->coord[idx]) < 0.5)
          {

              wp_msg.x = pt.x;
              wp_msg.y = pt.y;
              cout << "STATE: Traveling to the next waypoint" << endl;
          }
      }


          switch(cruiseAltitude)
          {
          case(1):
              wp_msg.z = baseAltitude + myId;
              break;
          case(-1):
              wp_msg.z = baseAltitude + myId;
              break;
          default:
              wp_msg.z = baseAltitude;
              break;
          }

      if(coll)
        cout << "UAV " << myId << " climbing to " << wp_msg.z << endl;

      waypoint_pub.publish(wp_msg);

      xp = 2.0*z*tan(fabs(av/2.0));
      yp = 2.0*z*tan(fabs(ah/2.0));

      // The camera is offset 2" from the drone center in the x-axis
      float x1 = x - 2.0*0.0254 - xp/2.0;
      float x2 = x - 2.0*0.0254 + xp/2.0;
      float y1 = y - yp/2.0;
      float y2 = y + yp/2.0;

      Mat visimagesc = Mat::zeros(500, 500, CV_8UC3);
      um->drawMap(visimagesc);
      pf->drawParticles(*um, visimagesc);
      Point uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x2,y2));
      Point uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x1,y1));
      rectangle(visimagesc, uv1, uv2, CV_RGB(255, 0, 0), 3);
      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(wp_msg.x, wp_msg.y));
      circle(visimagesc, uv1, 10, CV_RGB(0,255,0));

//      um->coord.push_back(Point2d(x,y));

      for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
      {
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[index[source(*ei, G)]]);
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[index[target(*ei, G)]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(255,255,255), 1);
          cv::circle(visimagesc, uv2, 3, CV_RGB(255,255,255));
      }

      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x,y));
      uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x+cos(proj),y+sin(proj)));
      cv::line(visimagesc, uv1, uv2, CV_RGB(0,0,255), 3);

      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[idxpath[0]]);
      for(int i=1; i < idxpath.size(); i++)
      {          
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, um->coord[idxpath[i]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(0,255,0), 2);
          uv1 = uv2;
      }

      for(int j=0; j < otherUAVst.size(); j++)
      {
          pt = Point2d(otherUAVst[j][0], otherUAVst[j][1]);
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, pt);
          int rad = estimRad[j]*visimagesc.rows/(um->ne.x - um->sw.x);
          cv::circle(visimagesc, uv1, rad, CV_RGB(100,0,0), 2);
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

//      um->coord.pop_back();

      delete[] weight;
      delete[] Edge;
  }

  void computeWaypoint(const ros::TimerEvent& te)
  {

      // Random tree
      typedef adjacency_list<vecS, vecS, directedS, \
              property<vertex_index_t, int>, property<edge_weight_t, double> > Graph;
      typedef property_map<Graph, edge_weight_t>::type WeightMap;
      typedef graph_traits<Graph >::vertex_descriptor vertex_descriptor;
      typedef property_map<Graph, vertex_index_t>::type IndexMap;

      bool onEdge=false, onNode=false;

      vector<Point2d> coord;

      int Nlay = 5;
      int children[] = {36, 1, 1, 1, 1};

      Graph G(181);
//      WeightMap wmap = get(edge_weight, G);
      double weight;

      coord.push_back(Point2d(x,y));
      double r = 1;

      int depth = 0;
      vector<int> chList;
      vector<int> pList;
      vector<double> aList;

      int vertex_id = 0;
      pList.push_back(0);

      Graph::edge_descriptor e; bool inserted;


      Point2d pt;

      double proj = 0;
      Vec3d vel(x-px, y-py, z-pz);
      proj = atan2(vel[1],vel[0]);

      while(depth < Nlay)
      {
          chList.clear();
          for(int k=0; k < pList.size(); k++)
          {
              for(int i=0; i < children[depth]; i++)
              {
                  vertex_id++;
                  chList.push_back(vertex_id);


                  pt = coord[pList[k]];

                  double angle;
                  if(depth == 0)
                  {
//                      angle = M_PI*(2*gsl_rng_uniform(RNG)-1.0);
                      angle = proj + gsl_ran_gaussian(RNG, M_PI/2.0);
                  }
                  else
                  {
                      angle = aList[pList[k-1]];
                      angle += M_PI*(2*gsl_rng_uniform(RNG)-1.0)/(double)(16*depth);
                  }
                  pt.x += r*cos(angle);
                  pt.y += r*sin(angle);
                  coord.push_back(pt);
                  aList.push_back(angle);
                  float x1 = pt.x - 2.0*0.0254 - xp/2.0;
                  float x2 = pt.x - 2.0*0.0254 + xp/2.0;
                  float y1 = pt.y - yp/2.0;
                  float y2 = pt.y + yp/2.0;
                  Point2d ul(x2,y2);
                  Point2d br(x1,y1);
                  double sc = 0.0;
                  double coll = 0.0;
                  for(int j=0; j < pf->N; j++)
                  {
                      pt = um->ep2coord((int)pf->pp[j][0], pf->pp[j][1]);
                      if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                          sc += pf->w[j];
                  }
                  pt = coord[pList[k]];
                  pt.x += r*cos(angle);
                  pt.y += r*sin(angle);
                  x1 = pt.x - 0.5;
                  x2 = pt.x + 0.5;
                  y1 = pt.y - 0.5;
                  y2 = pt.y + 0.5;
                  ul = Point2d(x2,y2);
                  br = Point2d(x1,y1);
                  for(int j=0; j < otherUAVst.size(); j++)
                  {
                      double estimRad = 0.0;
                      estimRad = norm(otherUAVvel[j]);
                      if(sqrt(pow(pt.x-otherUAVst[j][0],2) \
                         + pow(pt.y-otherUAVst[j][1],2)) < estimRad)
                         coll += 100.0;
                  }
                  if(1-sc > 0)
                    weight = 1-sc + coll;//+ 0.01*fabs(angle-proj)/M_PI;
                  else
                    weight = coll;
                  tie(e, inserted) = add_edge(pList[k], vertex_id, weight, G);
              }
          }
          pList = chList;
          depth++;
      }

      IndexMap index = get(vertex_index, G);
      vector<double> d(num_vertices(G));
      vector<vertex_descriptor> p(num_vertices(G));
      vertex_descriptor s = *(vertices(G).first);
      dijkstra_shortest_paths(G, s, predecessor_map(&p[0]).distance_map(&d[0]));

      int minidx = chList[0];
      double mindist = d[minidx];

      graph_traits<Graph>::vertex_iterator vi, vend;
      for(tie(vi, vend) = vertices(G); vi!=vend; ++vi)
      {
          for(int i=0; i < chList.size(); i++)
          {
              if(d[*vi] < mindist && chList[i] == index[*vi])
              {
                  minidx = index[*vi];
                  mindist = d[*vi];
              }
          }
      }

      cout << "Minimum distance is " << mindist << " to node " << minidx << endl;

      int vidx = minidx;
      vector<int> idxpath;
      idxpath.push_back(vidx);
      while(vidx != 0)
      {
          idxpath.push_back(p[vidx]);
          vidx = p[vidx];
      }

      cout << "Da path" << endl;
      for(vector<int>::iterator it=idxpath.begin(); it!=idxpath.end(); ++it)
      {
          cout << *it << ' ';
      }

      pt = coord[idxpath[0]];
      if(mindist > 99)
      {
          pt = Point2d(x,y);
          ROS_INFO_STREAM("There is no escape for the drone.");
      }

      pt = um->coord[idxpath[idxpath.size()-2]];
      bool coll = false;
      bool climb = false;
      for(int i=0; i < otherUAVst.size(); i++)
      {
          if(norm(Point2d(x,y) - Point2d(otherUAVst[i][0], otherUAVst[i][1])) < collisionRadius)
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
      double distance;
      tie(idx, distance) = findClosestNode(Point2d(x,y));

      if(!isUAVinGraph)
      {
          tie(idx, distance) = findClosestNode(Point2d(x,y));
          if(norm(Point2d(x,y) - um->coord[idx]) > 0.1 && !targetFound)
          {
              wp_msg.x = um->coord[idx].x;
              wp_msg.y = um->coord[idx].y;
              wp_msg.z = baseAltitude;
              cout << "STATE: Approaching the road network" << endl;
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
              cout << "STATE: Target detected, following it." << endl;
          }
          else if(norm(Point2d(x,y) - um->coord[idx]) < 0.5)
          {

              wp_msg.x = pt.x;
              wp_msg.y = pt.y;
              cout << "STATE: Traveling to the next waypoint" << endl;
          }
      }


          switch(cruiseAltitude)
          {
          case(1):
              wp_msg.z = baseAltitude + myId;
              break;
          case(-1):
              wp_msg.z = baseAltitude + myId;
              break;
          default:
              wp_msg.z = baseAltitude;
              break;
          }

      if(coll)
        cout << "UAV " << myId << " climbing to " << wp_msg.z << endl;

      waypoint_pub.publish(wp_msg);

      xp = 2.0*z*tan(fabs(av/2.0));
      yp = 2.0*z*tan(fabs(ah/2.0));

      // The camera is offset 2" from the drone center in the x-axis
      float x1 = x - 2.0*0.0254 - xp/2.0;
      float x2 = x - 2.0*0.0254 + xp/2.0;
      float y1 = y - yp/2.0;
      float y2 = y + yp/2.0;

      Mat visimagesc = Mat::zeros(500, 500, CV_8UC3);
      um->drawMap(visimagesc);
      pf->drawParticles(*um, visimagesc);
      Point uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x2,y2));
      Point uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x1,y1));
      rectangle(visimagesc, uv1, uv2, CV_RGB(255, 0, 0), 3);
      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(wp_msg.x, wp_msg.y));
      circle(visimagesc, uv1, 10, CV_RGB(0,255,0));

      std::cout << "edges(g) = ";
      graph_traits<Graph>::edge_iterator ei, ei_end;
      for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
      {
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, coord[index[source(*ei, G)]]);
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, coord[index[target(*ei, G)]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(255,255,255), 1);
          cv::circle(visimagesc, uv2, 3, CV_RGB(255,255,255));
      }

      uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x,y));
      uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, Point2d(x+cos(proj),y+sin(proj)));
      cv::line(visimagesc, uv1, uv2, CV_RGB(0,0,255), 3);

      for(int i=0; i < idxpath.size()-1; i++)
      {
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, coord[idxpath[i]]);
          uv2 = um->xy2uv(visimagesc.rows, visimagesc.cols, coord[idxpath[i+1]]);
          cv::line(visimagesc, uv1, uv2, CV_RGB(0,255,0), 2);
      }

      for(int j=0; j < otherUAVst.size(); j++)
      {
          pt = Point2d(otherUAVst[j][0], otherUAVst[j][1]);
          uv1 = um->xy2uv(visimagesc.rows, visimagesc.cols, pt);
          int rad = collisionRadius*visimagesc.rows/(um->ne.x - um->sw.x);
          cv::circle(visimagesc, uv1, rad, CV_RGB(255,0,0), 2);
          rad = detectionRadius*visimagesc.rows/(um->ne.x - um->sw.x);
          cv::circle(visimagesc, uv1, rad, CV_RGB(100,100,100), 2);
      }

      imshow("pdf", visimagesc);
      waitKey(3);


  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanner");
  pathPlanner dr;
  ros::spin();
  return 0;
}

