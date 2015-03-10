#include "targetstateestimator.h"

targetStateEstimator::targetStateEstimator(int rows, int cols, float alpha, float beta, float mppx, float mppy)
{
    occgrid.create(rows, cols, CV_32F);
    occgrid = Scalar(1/float(rows*cols));
    this->alpha = alpha;
    this->beta = beta;
    this->randproc = false;
    this->stddev = 0;
}

targetStateEstimator::targetStateEstimator(int rows, int cols, float alpha, float beta, float mppx, float mppy, double stddev)
{
    occgrid.create(rows, cols, CV_32F);
    occgrid = Scalar(1/float(rows*cols));
    this->alpha = alpha;
    this->beta = beta;
    this->randproc = true;
    this->stddev = stddev;
}

targetStateEstimator::~targetStateEstimator()
{
}

void targetStateEstimator::predictGrid()
{
    if(this->randproc == false)
        return;
    GaussianBlur(occgrid, occgrid, Size(0,0), this->stddev, this->stddev);
    float factor = sum(occgrid)[0];
    //cout << "Factor: " << factor << endl;
    if(factor > 1e-100)
        occgrid = occgrid*(1.0/factor);
    else
    {
        cout << "The target is not in the search area" << endl;
        occgrid = Scalar(1/float(occgrid.rows*occgrid.cols));
    }
}

void targetStateEstimator::updateGrid(Rect &area, bool measurement)
{
    Mat mask, fov;
    mask = Mat::zeros(occgrid.rows, occgrid.cols, CV_32F);
    Rect iarea = area;
    if(iarea.x < 0) iarea.x = 0;
    if(iarea.y < 0) iarea.y = 0;
    if(iarea.width+iarea.x-1 >= occgrid.cols)
      iarea.width = occgrid.cols - iarea.x;
    if(iarea.height+iarea.y-1 >= occgrid.rows)
      iarea.height = occgrid.rows - iarea.y;
    if(iarea.x >= occgrid.cols || iarea.y >= occgrid.rows || iarea.x+iarea.width < 0 \
         || iarea.y+iarea.height < 0)
    {
        cout << "Sensor is outside of the search domain" << endl;
        return;
    }
    fov = mask(iarea);
    fov = Scalar(1.0);
    
    
    if(measurement)
    {
        mask = Scalar(beta);
        fov = Scalar(alpha);
    }
    else
    {
        mask = Scalar(1-beta);
        fov = Scalar(1-alpha);
    }
    //cout << mask << endl;
    occgrid = occgrid.mul(mask);
    float factor = sum(occgrid)[0];
    //cout << "Factor: " << factor << endl;
    if(factor > 1e-100)
        occgrid = occgrid*(1.0/factor);
    else
    {
        cout << "The target is not in the search area" << endl;
        occgrid = Scalar(1/float(occgrid.rows*occgrid.cols));
    }

    // Floating point arithmetic requires a threshold!
//    for(int i=0; i < occgrid.rows; i++)
//    {
//        for(int j=0; j < occgrid.cols; j++)
//        {
//            if(occgrid.at<float>(i,j) < 1e-9)
//                occgrid.at<float>(i,j) = 0.0;
//        }
//    }
//    float factor = sum(occgrid)[0];
//    occgrid = occgrid*(1.0/factor);

    // What follows is an attempt to deal with zero-entropy grids
    double max;
    minMaxLoc(occgrid, NULL, &max);
    if(fabs(max - 1) < 1e-6)
    {
        cout << "Singleton grid estimate reached, smoothing" << endl;
        Mat newgrid = occgrid.clone();
        GaussianBlur(occgrid, newgrid, Size(5, 5), 0, 0, BORDER_CONSTANT);
        factor = sum(newgrid)[0];
        occgrid = newgrid*(1.0/factor);
    }
}

Mat targetStateEstimator::getGrid()
{
    return occgrid;
}

void targetStateEstimator::MLE()
{
    Mat xdist = Mat::zeros(1, occgrid.cols, CV_32F);
    Mat ydist = Mat::zeros(1, occgrid.rows, CV_32F);
    for(int i=0; i < occgrid.rows; i++)
    {
        for(int j=0; j < occgrid.cols; j++)
        {
            ydist.at<float>(0,i) += occgrid.at<float>(i,j);
            xdist.at<float>(0,j) += occgrid.at<float>(i,j);
        }
    }
    mean[0] = 0.0; // x coord estimate
    mean[1] = 0.0; // y
    std[0] = 0.0;
    std[1] = 0.0;
    for(int i=0; i < occgrid.rows; i++)
    {
        mean[1] += ydist.at<float>(0,i)*i;
        std[1] += ydist.at<float>(0,i)*i*i;
    }
    for(int j=0; j < occgrid.cols; j++)
    {
        mean[0] += xdist.at<float>(0,j)*j;
        std[0] += xdist.at<float>(0,j)*j*j;
    }
    //cout << std << endl;
    std[0] -= mean[0]*mean[0];
    std[1] -= mean[1]*mean[1];
    std[0] = sqrt(std[0]);
    std[1] = sqrt(std[1]);

}

