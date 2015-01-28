// Finding a target as a color blob
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015


#ifndef _TGTSTATE_H_
#define _TGTSTATE_H_

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

class targetStateEstimator
{
	Mat occgrid;
	float alpha, beta;
public:
	Vec2f mean;
	Vec2f std;
	targetStateEstimator(int rows, int cols, float alpha, float beta, float mppx, float mppy);
	~targetStateEstimator();
	void updateGrid(Rect &area, bool measurement);
	Mat getGrid();
	void  MLE();
};


#endif

