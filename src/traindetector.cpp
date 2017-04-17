// Target detector trainer
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2016

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;

class trainDetector
{
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub;
    ros::Subscriber object_sub;
    ros::Subscriber tgtpose_sub;
    geometry_msgs::Vector3 wp_msg;
    geometry_msgs::Pose targetPosition;

    double alpha, beta;
    bool tgtPresent;
    vector<float> altitudes;
    vector<unsigned long int> posD, negD, totalD;
    double minA, maxA;
    double droneA;

public:
    trainDetector()
    {

        pose_sub = nh_.subscribe("/ardrone/pose", 2, &trainDetector::Callback, this);
        object_sub = nh_.subscribe("/objectdetected", 2, &trainDetector::detectionCallback, this);
        tgtpose_sub = nh_.subscribe("/objectpose", 1, &trainDetector::targetPositionCallback, this);

        ROS_INFO_STREAM("Target detector trainer.");

        int bins;
        nh_.param("/train_detector/targetpresent", tgtPresent, true);
        nh_.param("/train_detector/min_altitude", minA, 0.5);
        nh_.param("/train_detector/max_altitude", maxA, 2.0);
        nh_.param("/train_detector/bins", bins, 20);

        if(tgtPresent)
            ROS_INFO_STREAM("Assuming that the target is present in the camera view.");
        else
            ROS_INFO_STREAM("Assuming that the target is not present in the camera view");



        for(int i=0; i < bins; i++)
        {
            altitudes.push_back(i*(maxA-minA)/(float)bins + minA);
            posD.push_back(0);
            negD.push_back(0);
            totalD.push_back(0);
        }
    }

    ~trainDetector()
    {
        ROS_INFO_STREAM("Writing data to disk as traindata.txt");
        ofstream ofile;
        ofile.open("traindata.txt", ios::out);

        for(int i=0; i < altitudes.size(); i++)
        {
            if(totalD[i] > 0)
                ofile << altitudes[i] << " " << (double)posD[i]/(double)totalD[i] << " " << (double)negD[i]/(double)totalD[i] << endl;
            else
                ofile << altitudes[i] << " " << 0.0 << " " << 0.0 << endl;
        }

        ofile.close();
    }

    void targetPositionCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        targetPosition = *msg;
    }

    void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        droneA = msg->pose.position.z;
        droneA = 1;
    }

    void detectionCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        float altDist;
        float minAdiff = 1e3;
        int minidx = 0;

        for(int i=0; i < altitudes.size(); i++)
        {
            altDist = fabs(altitudes[i] - droneA);
            if(altDist < minAdiff)
            {
                minAdiff = altDist;
                minidx = i;
            }
        }

        cout << droneA << endl;
        cout << altitudes[minidx] << endl;

        if(msg->data == true)
            posD[minidx]++;
        else
            negD[minidx]++;

        totalD[minidx]++;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traindetector");
    trainDetector dr;
    ros::spin();
    return 0;
}
