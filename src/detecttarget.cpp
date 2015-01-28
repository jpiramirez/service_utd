// Finding a target as a color blob
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015


#include "colortarget.h"

colortarget::colortarget()
{
	calibMatrix.create(3, 3, CV_64F);

}

colortarget::~colortarget()
{
}

void colortarget::computeStateWithColorBlob(Mat pov)
{
	double colorDist;
	Vec3f colorVec;
	int i;
	double f;
	Rect ROIrect;
	
	f = calibMatrix.at<double>(0,0);

	colorVec[0] = color.val[0];
	colorVec[1] = color.val[1];
	colorVec[2] = color.val[2];
	for(i=0; i < pov.rows; i++)
		for(int j=0; j < pov.cols; j++)
		{
			colorDist = 0.0;
			for(int k=0; k < 3; k++)
				colorDist += pow(pov.at<Vec3f>(i,j)[k]-colorVec[k], 2.0f);
			colorDist = sqrt(colorDist);
			if(colorDist < 120)
				pov.at<Vec3f>(i,j) = Vec3f(255,255,255);
			else
				pov.at<Vec3f>(i,j) = Vec3f(0,0,0);
		}
	Mat povGray;
	cvtColor(pov, povGray, CV_RGB2GRAY);
	povGray.convertTo(povGray, CV_8UC1);
	vector<vector<Point> > contours;
	findContours(povGray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Safety measure: if no blob is located, stop movement (set every
	// controller gain to zero).
	if(contours.size() < 1)
	{
		targetAcquired = false;
		return;
	}

	vector<vector<Point> >::iterator citer;
	double maxarea = 0;
	for(citer = contours.begin(); citer != contours.end(); citer++)
	{
		Moments mset;
		mset = moments(*citer, false);
		if(mset.m00 > maxarea)
		{
			if(citer->size() < 5)
				continue;
			maxarea = mset.m00;
			position[0] = mset.m10/mset.m00;
			position[1] = mset.m01/mset.m00;
			ROIrect = boundingRect(*citer);
			//ROIrect = fitEllipse(*citer);
		}
	}

	if(maxarea < 100)
	{
		targetAcquired = false;
		return;
	}
	
	tgtLoc.x = position[0];
	tgtLoc.y = position[1];
	tgtRect = ROIrect;

	position[2] = f*tgtSize/(double)ROIrect.width;
	//filter[fcount] = f*targetSize/(double)ROIrect.size.height;
	//fcount++;
	//fcount = fcount % 20;
	//z = f*targetSize/(double)ROIrect.size.height;
	//z = 0;
	//for(i=0; i < 20; i++)
	//	z += filter[i];
	//z /= 20.0;
	position[0] = -(position[0] - pov.cols/2.0)/f;
	position[1] = -(position[1] - pov.rows/2.0)/f;
	position[0] *= position[2];
	position[1] *= position[2];
	targetAcquired = true;
}




