// Pose estimation using color blobs.
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012


#ifndef _VISTARGET_H_
#define _VISTARGET_H_

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <geometry_msgs/Pose.h>

#ifndef M_PI
#define M_PI 3.1415926535897932385
#endif

using namespace cv;
using namespace std;

class colortarget
{
	Mat internal;
	double blobDistance(Moments a, Moments b);
	double pointDistance(Point2f a, Point2f b);
	bool targetAcquired;


		
public:
	Vec3d position; // X, Y, Z translation
	Vec4d orientation; // (w,x,y,z) quaternion
	int f;
	Mat calibMatrix;
	vector<double> distCoeff;
	Scalar color;
	Point tgtLoc;
	Rect tgtRect;

	double tgtSize;
	colortarget();
	~colortarget();
	void computeStateWithColorBlob(Mat pov);
	bool targetFound() { return targetAcquired; };
};


#endif

