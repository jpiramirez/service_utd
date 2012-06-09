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

class vistarget
{
	Mat internal;
	vector<Point2f> markers;
	double blobDistance(Moments a, Moments b);
	double pointDistance(Point2f a, Point2f b);
	bool targetAcquired;
	Vec3d position;
	Vec4d orientation;
	
public:
	double tgtSize;
	vistarget();
	~vistarget();
	void computeMarkerLocations(Mat image);
	vector<Point2f> getMarkers() { return markers; };
	void estimatePose();
};


#endif

