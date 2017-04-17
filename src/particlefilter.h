#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

// Particle filter estimator
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016



#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "urbanmap.hpp"

#ifndef M_PI
#define M_PI 3.1415926535897932385
#endif

#define TWODIM 0
#define GRAPH  1

using namespace cv;
using namespace std;

class particleFilter
{
protected:
    uchar pfType;
    const gsl_rng_type *T;
    gsl_rng *r;

public:
    vector<Vec3f> pp;
    vector<double> w;
    Vec3f upperbnd, lowerbnd;
    int N;
    float alpha, beta;
    bool randproc;
    float stddev;
    double maxvel;

    particleFilter(int N, float alpha, float beta, Vec3f upperbnd, Vec3f lowerbnd, double stddev);
    particleFilter(int N, float alpha, float beta, urbanmap um, double maxvel);
    ~particleFilter();
    void reset(urbanmap um);
    void predict();
    void update(Point2f ul, Point2f br, bool measurement);
    void update(Point2f mean, Mat &cov, bool measurement);
    void update(urbanmap um, Point2d ul, Point2d br, bool measurement);
    void update(urbanmap um, Point2d pos, double stddev);
    void predict(urbanmap um, double Ts);
    void resample();
    void display();
    void drawParticles(urbanmap um, Mat map);
    Vec3f meanEstim(urbanmap um);
    //Mat getGrid();
    void  MLE();
};





#endif // PARTICLEFILTER_H
