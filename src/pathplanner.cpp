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

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace boost;

class pathPlanner
{
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub;
  ros::Publisher map_pub;
  ros::Publisher searchmap_pub;
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

  double collisionRadius, detectionRadius;

  const gsl_rng_type *T;
  gsl_rng *RNG;

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
    pf = new particleFilter(4000, alpha, beta, *um, 0.1);

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

    timer = nh_.createTimer(ros::Duration(0.2), &pathPlanner::computeWaypoint, this);
    timerpf = nh_.createTimer(ros::Duration(0.1), &pathPlanner::broadcastParticles, this);

    namedWindow("pdf");

    ptime = ros::Time::now();
    ctime = ros::Time::now();

    targetFound = false;
    confThresh = 0.95;
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

    nh_.param<double>("collision_radius", collisionRadius, 1);
    nh_.param<double>("detection_radius", detectionRadius, 2);

    collided = false;
    T = gsl_rng_mt19937;
    RNG = gsl_rng_alloc(T);
    gsl_rng_env_setup();

  }

  ~pathPlanner()
  {
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

      px = x, py = y, pz = z;

      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;

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


      ptime = ctime;
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

        ctime = ros::Time::now();
        d = ctime - pestim[callerId];
        double t = d.toSec();

        std::string myname = nh_.getNamespace();
        // This gets rid of the leading "//uav" to retrieve only the UAV number. This requires the user to
        // specify the namespace as "uav##"
        myname = myname.erase(0,5);
        int myId = atoi(myname.c_str());
        if(myId == detectedUAV)
            return;

        double distanceToUAV = 0;
        distanceToUAV += pow(msg->pose.position.x-x, 2.0);
        distanceToUAV += pow(msg->pose.position.y-y, 2.0);
        distanceToUAV += pow(msg->pose.position.z-z, 2.0);
        distanceToUAV = sqrt(distanceToUAV);

        otherUAVpst[callerId] = otherUAVst[callerId];
        otherUAVst[callerId] = Vec3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        otherUAVvel[callerId] = (otherUAVst[callerId]-otherUAVpst[callerId])/t;




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
      pfset[detectedUAV] = *msg;

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

      if(!broadcast || msg->N != pf->N)
          return;

      ROS_INFO_STREAM(nh_.getNamespace() << " fusing PFs");

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

      cout << "Particle set size: " << fusedpp.size() << endl;

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


      int k = 0;
      for(int i=0; i < pf->N; i++)
      {
          u.push_back((gsl_rng_uniform(RNG) + i)/(double)pf->N);
          while(wc[k] < u[i])
              k++;
          ind[i] = k;
      }

//      for(int i=0; i < pf->N; i++)
//      {
//          ind[i] = gsl_rng_uniform_int(RNG, pf->N);
//      }

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

//  void computeWaypoint(const ros::TimerEvent& te)
//  {
//      Mat score = Mat::zeros(1, pf->N/10, CV_32F);
//      xp = 2.0*z*tan(fabs(av/2.0));
//      yp = 2.0*z*tan(fabs(ah/2.0));
//      int k;
//      Point maxIdx;
//      Point2d pt;

//      Mat vel(otherUAVvel.size(), 3, CV_64F);

//      for(int i=0; i < otherUAVvel.size(); i++)
//      {
//          for(k=0; k < 3; k++)
//          {
//              vel.at<double>(i, k) = otherUAVvel[i][k];
//          }
//      }

//      //Mat sol = Mat::zeros(3, 1, CV_64F);
//      Vec3d toGoal(0,0,0);
//      Vec3d sol(0,0,0);
//      cv::SVD::solveZ(vel, sol);

//      for(int i=0; i < pf->N/10; i++)
//      {
//          double sc = 0.0;

//          pt = um->ep2coord((int)pf->pp[i*10][0], pf->pp[i*10][1]);
//          float x1 = pt.x - 2.0*0.0254 - xp/2.0;
//          float x2 = pt.x - 2.0*0.0254 + xp/2.0;
//          float y1 = pt.y - yp/2.0;
//          float y2 = pt.y + yp/2.0;
//          Point2d ul(x2,y2);
//          Point2d br(x1,y1);
//          for(k=0; k < pf->N; k++)
//          {
//              pt = um->ep2coord((int)pf->pp[k][0], pf->pp[k][1]);
//              if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
//                  sc += pf->w[k];
//          }

//          toGoal = Vec3d(pt.x-x, pt.y-y, 0);
//          score.at<float>(0,i) = fabs(sc-Copt) + toGoal.dot(sol) + cv::norm(toGoal);
//      }

//      minMaxLoc(score, NULL, NULL, NULL, &maxIdx);
//      k = maxIdx.x;
//      pt = um->ep2coord((int)pf->pp[k][0], pf->pp[k][1]);

//      wp_msg.x = pt.x;
//      wp_msg.y = pt.y;
//      wp_msg.z = 0.5;
//      waypoint_pub.publish(wp_msg);


//  }
//  void computeWaypoint(const ros::TimerEvent& te)
//  {
//      Mat score = Mat::zeros(9, 9, CV_32F);
//      xp = 2.0*z*tan(fabs(av/2.0));
//      yp = 2.0*z*tan(fabs(ah/2.0));
//      int k;
//      Point maxIdx;
//      Point2d pt;
//      double r = 0.1;


//      for(int i=0; i < score.rows; i++)
//      {
//          for(int j=0; j < score.cols; j++)
//          {
//              double sc = 0.0;
//              double potential = 0.0;
//              double distance = 0.0;

//              pt.x = x + r*(double)(j-score.cols/2);
//              pt.y = y + r*(double)(i-score.rows/2);
//              float x1 = pt.x - 2.0*0.0254 - xp/2.0;
//              float x2 = pt.x - 2.0*0.0254 + xp/2.0;
//              float y1 = pt.y - yp/2.0;
//              float y2 = pt.y + yp/2.0;
//              Point2d ul(x2,y2);
//              Point2d br(x1,y1);
//              for(k=0; k < pf->N; k++)
//              {
//                  pt = um->ep2coord((int)pf->pp[k][0], pf->pp[k][1]);
//                  if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
//                      sc += pf->w[k];
//              }
//              pt.x = x + r*(double)(j-score.cols/2);
//              pt.y = y + r*(double)(i-score.rows/2);
//              for(k=0; k < otherUAVst.size(); k++)
//              {
//                  distance = sqrt(pow(pt.x-otherUAVst[k][0],2.0) + pow(pt.y-otherUAVst[k][1],2.0));
//                  if(distance < collisionRadius)
//                    potential += distance;
////                  if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
////                      collnum++;
//              }

//              score.at<float>(i,j) = fabs(sc-Copt) - potential;
//          }
//      }

//      Point minLoc;
//      cv::minMaxLoc(score, NULL, NULL, &minLoc, NULL);
//      //k = maxIdx.x;
//      //pt = um->ep2coord((int)pf->pp[k][0], pf->pp[k][1]);
//      pt.x = x + r*(double)(minLoc.x-score.cols/2);
//      pt.y = y + r*(double)(minLoc.y-score.rows/2);

//      wp_msg.x = pt.x;
//      wp_msg.y = pt.y;
//      wp_msg.z = 0.5;
//      waypoint_pub.publish(wp_msg);


//  }

  void computeWaypoint(const ros::TimerEvent& te)
  {

      // Random tree
      typedef adjacency_list<vecS, vecS, directedS, \
              property<vertex_index_t, int>, property<edge_weight_t, double> > Graph;
      typedef property_map<Graph, edge_weight_t>::type WeightMap;
      typedef graph_traits<Graph >::vertex_descriptor vertex_descriptor;
      typedef property_map<Graph, vertex_index_t>::type IndexMap;


      vector<Point2d> coord;

      int Nlay = 3;
      int children[] = {36, 1, 1};

      Graph G(109);
//      WeightMap wmap = get(edge_weight, G);
      double weight;

      coord.push_back(Point2d(x,y));
      double r = 0.4;

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
                      if(sqrt(pow(pt.x-otherUAVst[j][0],2) \
                         + pow(pt.y-otherUAVst[j][1],2)) < collisionRadius)
                         coll += 100.0;
                  }
                  weight = 1-sc + coll+ 0.01*fabs(angle-proj)/M_PI;
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

      pt = coord[idxpath[idxpath.size()-2]];
      if(mindist > 99)
      {
          pt = Point2d(x,y);
          ROS_INFO_STREAM("There is no escape for the drone.");
      }
      wp_msg.x = pt.x;
      wp_msg.y = pt.y;
      wp_msg.z = 0.5;
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

