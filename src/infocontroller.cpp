// Information based planner or controller
// Needs a set-point controller in place
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
#include "targetstateestimator.h"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;
using namespace cv;

class infoController
{
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub;
  ros::Publisher map_pub;
  ros::Publisher searchmap_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber object_sub;
  ros::Subscriber tgtpose_sub;
  geometry_msgs::Vector3 wp_msg;
  geometry_msgs::Pose targetPosition;
  service_utd::ProbMap pm_msg, sm_msg;
  float x, y, z;
  ros::Time ptime, ctime;
  ros::Duration d;
  float px, py, pz;
  float setx, sety, setz;
  targetStateEstimator *tse;
  float fv, fh;
  float av, ah;
  int nsqx, nsqy;
  ros::Timer timer;
  Rect area;
  int seqnum;
  ofstream timestamps;
  float xp, yp;
  float gridsizex, gridsizey;
  float squaresize;
  // These matrices store the world coordinates of the target pdf grid
  Mat xcoords;
  Mat ycoords;
  double Copt;
  vector<double> wpx, wpy, wpz;
  int wpcount;
  vector<float> xh, yh, zh;
  double alpha, beta;
  bool detected;

  bool targetFound;
  double confThresh;
  double detectConfidence;

  bool waypointNav;
  
public:
  infoController()
  {
    waypoint_pub = nh_.advertise<geometry_msgs::Vector3>("/ardrone/setpoint", 2);
    map_pub = nh_.advertise<service_utd::ProbMap>("/probmap", 2);
    searchmap_pub = nh_.advertise<service_utd::ProbMap>("/searchmap", 2);
    pose_sub = nh_.subscribe("/ardrone/pose", 2, &infoController::Callback, this);
    object_sub = nh_.subscribe("/objectdetected", 2, &infoController::detectionCallback, this);
    tgtpose_sub = nh_.subscribe("/objectpose", 1, &infoController::targetPositionCallback, this);

    ROS_INFO_STREAM("Set point controller initialized.");
    x = 0.0;
    y = 0.0;
    z = 0.0;
    px = 0;
    py = 0;
    pz = 0;
    for(int i=0; i < 10; i++)
    {
        xh.push_back(0);
        yh.push_back(0);
        zh.push_back(0);
    }
    setx = 0;
    sety = 0;
    setz = 1.4;
    ptime = ros::Time::now();
    ctime = ros::Time::now();
    gridsizex = 4;
    gridsizey = 4;
    squaresize = 0.05;
    nsqx = floor(gridsizex/squaresize);
    nsqy = floor(gridsizey/squaresize);
    nh_.param<double>("/alpha", alpha, 0.8);
    nh_.param<double>("/beta", beta, 0.2);
    tse = new targetStateEstimator(nsqy, nsqx, 0.8, 0.2, squaresize, squaresize, 0.1);
    //tse = new targetStateEstimator(nsqy, nsqx, 0.8, 0.2, squaresize, squaresize);
    cout << "No diffusion added" << endl;

    ROS_INFO_STREAM("PDF grid is " << nsqy << "x" << nsqx);

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
    area.x = 0;
    area.y = 0;
    area.width = 0;
    area.height = 0;

    xcoords.create(nsqy, nsqx, CV_32F);
    ycoords.create(nsqy, nsqx, CV_32F);

    timer = nh_.createTimer(ros::Duration(0.2), &infoController::computeWaypoint, this);

    namedWindow("pdf");

    targetFound = false;
    confThresh = 0.95;
    detectConfidence = 0;

    detected = false;

    nh_.param("/waypointNav", waypointNav, false);

    if(waypointNav)
    {
        ROS_INFO_STREAM("Loading waypoint list");
        std::string xpts;
        nh_.param<std::string>("/waypoints/x", xpts, "0 1 1 0");
        std::string ypts;
        nh_.param<std::string>("/waypoints/y", ypts, "0 0 1 1");
        std::string zpts;
        nh_.param<std::string>("/waypoints/z", zpts, "1 1 1 1");
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

  }

  ~infoController()
  {
  }

  void targetPositionCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
      targetPosition = *msg;
  }

  void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      double xsum = 0, ysum = 0, zsum = 0;
      for(int i=0; i < xh.size()-1; i++)
      {
          xsum += xh[i];
          ysum += yh[i];
          zsum += zh[i];
          xh[i] = xh[i+1];
          yh[i] = yh[i+1];
          zh[i] = zh[i+1];
      }
      xsum += xh[xh.size()-1];
      ysum += yh[yh.size()-1];
      zsum += zh[zh.size()-1];
      x = msg->pose.position.x;
      y = msg->pose.position.y;
      z = msg->pose.position.z;
      xh[xh.size()-1] = x;
      yh[yh.size()-1] = y;
      zh[zh.size()-1] = z;
      px = xsum/(xh.size());
      py = ysum/(yh.size());
      pz = zsum/(zh.size());

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
      
      
      
      // Our grid is 2x2 meters, centered at (0,0) on the floor
      area.x = floor(nsqx/2-nsqx*y2/gridsizex);
      area.y = floor(nsqy/2-nsqy*x2/gridsizey);

      //area.width = floor(nsqx*yp/gridsizex);
      //area.height = floor(nsqy*xp/gridsizey);

      area.width = floor(yp/squaresize);
      area.height = floor(xp/squaresize);

      tse->predictGrid();
      if(targetFound)
      {
          Point meanp;
          meanp.x = -(targetPosition.position.y+y)*nsqx/(float)gridsizex + nsqx/2;
          meanp.y = -(targetPosition.position.x+x)*nsqy/(float)gridsizey + nsqy/2;
          Mat cov = 3*Mat::eye(Size(2,2), CV_32F);
          tse->updateGridGaussian(meanp, cov, true);
      }
      else
          tse->updateGrid(area, detected);
      tse->MLE();
      //cout << "Mean: " << tse->mean*(-2.0/float(nsqx))+Vec2f(1,1) << endl;
      //cout << "StdDev: " << tse->std*(-2.0/float(nsqx))+Vec2f(1,1) << endl;

      pm_msg.data.clear();
      pm_msg.header.seq = seqnum;
      pm_msg.header.stamp = ros::Time::now();
      pm_msg.header.frame_id = "map";
      pm_msg.width = nsqx;
      pm_msg.height = nsqy;
      
      Mat data = tse->getGrid();
      for(int i=0; i < data.rows; i++)
      {
          for(int j=0; j < data.cols; j++)
          {
              pm_msg.data.push_back(data.at<float>(i,j));
          }
      }

      // Visualizing the pdf
      if(targetFound)
      {
          double minval, maxval;
          minMaxLoc(data, &minval, &maxval);
          data = (255.0/maxval)*data;
          Mat probim = Mat::zeros(data.cols, data.rows, CV_8UC3);
          Mat tempim = Mat::zeros(data.cols, data.rows, CV_8UC1);
          int sfactor = 10;
          Mat visimagesc(data.rows*sfactor, data.cols*sfactor, CV_8UC3);
          data.convertTo(tempim, CV_8UC1);
          cvtColor(tempim, probim, CV_GRAY2BGR);
          resize(probim, visimagesc, visimagesc.size());
          imshow("pdf", visimagesc);
          waitKey(3);
      }

      map_pub.publish(pm_msg);

      seqnum++;
  }
  
  void computeWaypoint(const ros::TimerEvent& te)
  {
      // This is the real controller. Once we have a probability distribution for the target
      // location we send the drone to the most informative location. This requires the setpoint
      // controller to be running.

      if (area.width == 0 || area.height == 0)
      {
          return;
      }



      Mat M = tse->getGrid();
      Mat mask = Mat::ones(area.height, area.width, CV_32F);
      Mat result = Mat::zeros(M.rows, M.cols, CV_32F);
      filter2D(M, result, -1, mask, Point(-1,-1), 0, BORDER_CONSTANT);

      Mat pmask, fov;
      pmask = Mat::zeros(M.rows, M.cols, CV_32F);
      Rect iarea = area;
      if(iarea.x < 0) iarea.x = 0;
      if(iarea.y < 0) iarea.y = 0;
      if(iarea.width+iarea.x-1 >= M.cols)
        iarea.width = M.cols - iarea.x;
      if(iarea.height+iarea.y-1 >= M.rows)
        iarea.height = M.rows - iarea.y;
      if(iarea.x >= M.cols || iarea.y >= M.rows || iarea.x+iarea.width < 0 \
           || iarea.y+iarea.height < 0)
      {
          cout << "Sensor is outside of the search domain" << endl;
          return;
      }
      fov = pmask(iarea);
      fov = Scalar(1.0);

      Mat probxS = M.mul(pmask);
      double pxinS = sum(probxS)[0];

      //detectConfidence = alpha*pxinS / (alpha*pxinS + beta*(1-pxinS));

      if(detected)
      {
          detectConfidence = alpha*pxinS / (alpha*pxinS + beta*(1-pxinS));
      }
      else
      {
          detectConfidence = (1-alpha)*pxinS / ((1-alpha)*pxinS + (1-beta)*(1-pxinS));
      }

      if(detectConfidence > confThresh)
      {
          targetFound = true;
      }
      else
          targetFound = false;

      cout << "Detection confidence: " << detectConfidence << endl;

      // If the target has been declared as Found, then switch to minimize the position error
      // between the drone and the target.
      if(targetFound)
      {
          wp_msg.x = targetPosition.position.x+x;
          wp_msg.y = targetPosition.position.y+y;
          wp_msg.z = 1;
          waypoint_pub.publish(wp_msg);
          return;
      }


      Mat probvol = cv::abs(result - Copt);
      // We will now attempt to give more weight to the grid locations closer to the sensor.

      Mat distmask = Mat::zeros(M.rows, M.cols, CV_32F);
      Mat angmap = Mat::zeros(M.rows, M.cols, CV_32F);
      Mat zcross = Mat::zeros(M.rows, M.cols, CV_32F);
      double vx, vy, vmag;
      vx = x-px;
      vy = y-py;
      vmag = sqrt(vx*vx + vy*vy);



      for(int i=0; i < distmask.rows; i++)
      {
          for(int j=0; j < distmask.cols; j++)
          {
              ycoords.at<float>(i,j) = -squaresize*j+gridsizex/2.0   -y;
              xcoords.at<float>(i,j) = -squaresize*i+gridsizey/2.0   -x;
              distmask.at<float>(i,j) = sqrt(pow(xcoords.at<float>(i,j),2) + pow(ycoords.at<float>(i,j),2));
              zcross.at<float>(i,j) = copysign(1, vx*ycoords.at<float>(i,j) - vy*xcoords.at<float>(i,j));
          }
      }

      angmap = -vx*xcoords - vy*ycoords;
      angmap = angmap/(distmask*vmag);
      for(int i=0; i < angmap.rows; i++)
          for(int j=0; j < angmap.cols; j++)
              angmap.at<float>(i,j) = acos(angmap.at<float>(i,j));

      double maxval, minval;
      distmask = distmask.mul(distmask);
      minMaxLoc(distmask, &minval, &maxval);
      distmask = (distmask-minval) * 1.0/(maxval-minval);


      Mat projmap = zcross.mul(angmap);
      minMaxLoc(angmap, &minval, &maxval);
      if(fabs(maxval) > 1e-30)
          projmap = (projmap-minval) * 1.0/(maxval-minval);
      else
          projmap = Mat::zeros(M.rows, M.cols, CV_32F);

      Mat covermap(M.rows, M.cols, CV_32F);
      minMaxLoc(probvol, &minval, &maxval);
      if(fabs(maxval) > 1e-30)
          covermap = (probvol-minval) * 1.0/(maxval-minval);
      else
          covermap = Mat::zeros(M.rows, M.cols, CV_32F);

      Mat costfnc = 0.3*covermap + 0.1*projmap + 0.6*distmask;
      minMaxLoc(costfnc, NULL, &maxval);

      Point minloc;
      minMaxLoc(costfnc, NULL, NULL, &minloc, NULL);

      //cout << probvol << endl;
      cout << "Min loc " << minloc << endl;

      // Converting grid coordinates to world coordinates
      //wp_msg.x = -squaresize*minloc.y + gridsizey/2.0;
      //wp_msg.y = -squaresize*minloc.x + gridsizex/2.0;
      if(waypointNav == false)
      {
          wp_msg.x = xcoords.at<float>(minloc.y, minloc.x)+x;
          wp_msg.y = ycoords.at<float>(minloc.y, minloc.x)+y;
          wp_msg.z = 1;
          cout << "[" << wp_msg.x << "," << wp_msg.y << "]" << endl;
          cout << "Val: " << probvol.at<float>(minloc.y, minloc.x) << endl;
      }
      else
      {
          wp_msg.x = wpx[wpcount];
          wp_msg.y = wpy[wpcount];
          wp_msg.z = wpz[wpcount];
      }
      waypoint_pub.publish(wp_msg);

      sm_msg.data.clear();
      sm_msg.header.seq = seqnum;
      sm_msg.header.stamp = ros::Time::now();
      sm_msg.header.frame_id = "map";
      sm_msg.width = nsqx;
      sm_msg.height = nsqy;

      for(int i=0; i < costfnc.rows; i++)
      {
          for(int j=0; j < costfnc.cols; j++)
          {
              sm_msg.data.push_back(costfnc.at<float>(i,j));
          }
      }

      int sfactor = 10;
      Mat visimage(M.rows, M.cols, CV_8UC3);
      Mat visimagesc(M.rows*sfactor, M.cols*sfactor, CV_8UC3);

      Mat tempimg(M.rows, M.cols, CV_32FC3);


      minMaxLoc(costfnc, &minval, &maxval);
      if(fabs(maxval) > 1e-100)
          covermap = costfnc * 1.0/maxval;
      else
          covermap = Mat::zeros(M.rows, M.cols, CV_32F);
      cvtColor(covermap, tempimg, CV_GRAY2BGR);
      tempimg.convertTo(visimage, CV_8UC3, 255.0/(maxval-minval), -minval*255.0/(maxval-minval));
      visimage.at<Vec3b>(minloc.y, minloc.x)[0] = 0;
      visimage.at<Vec3b>(minloc.y, minloc.x)[1] = 0;
      visimage.at<Vec3b>(minloc.y, minloc.x)[2] = 255;
//      visimage.at<Vec3b>(floor(nsqy/2-nsqy*x/gridsizey), floor(nsqx/2-nsqx*y/gridsizex))[0] = 255;
//      visimage.at<Vec3b>(floor(nsqy/2-nsqy*x/gridsizey), floor(nsqx/2-nsqx*y/gridsizex))[1] = 0;
//      visimage.at<Vec3b>(floor(nsqy/2-nsqy*x/gridsizey), floor(nsqx/2-nsqx*y/gridsizex))[2] = 255;
      rectangle(visimage, area, Scalar(0,255,0));

      resize(visimage, visimagesc, visimagesc.size());

      cout << "grid is " << visimagesc.rows << "x" << visimagesc.cols << endl;
      imshow("pdf", visimagesc);
      waitKey(3);

      searchmap_pub.publish(sm_msg);
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_based_ctrl");
  infoController dr;
  ros::spin();
  return 0;
}
