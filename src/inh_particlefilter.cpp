#include "inh_particlefilter.hpp"

void inhParticleFilter::simpleUpdate(urbanmap um, Point2d ul, Point2d br, bool measurement)
{
    double wsum = 0.0;
    Point2d pt;
    this->pp = phist.getElement(0);
    vector<int> previdx;
    for(int i=0; i < N; i++)
        previdx.push_back(i);
    this->w = whist.getElement(0);

    for(int i=0; i < N; i++)
    {
        pt = um.ep2coord((int)pp[i][0], pp[i][1]);
        if(measurement)
        {
            if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                w[i] *= alpha;
            else
            {
                w[i] *= beta;
            }
        }
        else
        {
            if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                w[i] *= 1.0-alpha;
            else
                w[i] *= 1.0-beta;
        }

        wsum += w[i];
    }

//    if(wsum < 0.001)
//    {
//        reset(um);
//        return;
//    }

    double Neff = 0.0;
    for(int i=0; i < N; i++)
    {
        w[i] /= wsum;
        Neff += pow(w[i], 2.0);
    }

    Neff = 1.0/Neff;
    if(isnan(Neff))
        Neff = 0.0;
    double Ns = (double)N/10.0;

    if(Neff < Ns)
    {
        vector<double> u, wc;
        vector<int> ind;

        for(int i=0; i < N; i++)
        {
            u.push_back((gsl_rng_uniform(r) + i)/(double)N);
            if(i == 0)
                wc.push_back(w[0]);
            else
                wc.push_back(wc[i-1] + w[i]);
            ind.push_back(0);
        }


        int k = 0;
        for(int i=0; i < N; i++)
        {

            while(wc[k] < u[i])
                k++;
            ind[i] = k;
        }

        vector<Vec3f> npp;
        for(int i=0; i < N; i++)
            npp.push_back(Vec3f(0,0,0));
        for(int i=0; i < N; i++)
        {
            previdx[i] = ind[i];
            npp[i] = pp[ind[i]];
            w[i] = 1.0/(double)N;
        }

        pp = npp;
    }

    phist.pushElement(this->pp);
    pid.pushElement(previdx);
    whist.pushElement(w);
    tstamp.pushElement(timesig::now());
}

void inhParticleFilter::processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, int index, double alpha, double beta)
{
    if(index < 0)
        throw std::out_of_range("Negative indices are not allowed.");

    this->pp = phist[index];
    this->w = whist[index];
    double wsum = 0.0;
    Point2d pt;
    for(int i=0; i < N; i++)
    {
        pt = um.ep2coord((int)pp[i][0], pp[i][1]);
        if(measurement)
        {
            if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                w[i] *= alpha;
            else
            {
                w[i] *= beta;
            }
        }
        else
        {
            if(pt.x < ul.x && pt.y < ul.y && pt.x > br.x && pt.y > br.y)
                w[i] *= 1.0-alpha;
            else
                w[i] *= 1.0-beta;
        }

        wsum += w[i];
    }
    for(int i=0; i < N; i++)
    {
        w[i] /= wsum;
    }
    phist.putElement(this->pp, index);
    whist.putElement(this->w, index);

    vector<int> parindex;

    for(int i=index-1; i >= 0; i--)
    {
        parindex = pid[i];
        vector<double> oldw = whist[i+1];
        for(int j=0; j < N; j++)
        {
            w[j] = oldw[parindex[j]];
        }
        whist.putElement(w, i);
    }

    this->pp = phist[0];
    this->w = whist[0];
}

void inhParticleFilter::processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, int index)
{
    inhParticleFilter::processOOSM(um, ul, br, measurement, index, this->alpha, this->beta);
}

void inhParticleFilter::processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, timesig Ts)
{
    int idx = findIndex(Ts);
    if(idx < 0)
    {
        std::cout << "No sample matches the measurement time." << std::endl;
        return;
    }

    processOOSM(um, ul, br, measurement, idx);
}

void inhParticleFilter::processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, timesig Ts, double alpha, double beta)
{
    int idx = findIndex(Ts);
    if(idx < 0)
    {
        std::cout << "No sample matches the measurement time." << std::endl;
        return;
    }

    processOOSM(um, ul, br, measurement, idx, alpha, beta);
}

int inhParticleFilter::findIndex(timesig Ts)
{
  int idx = 0;
  while(idx < tstamp.size())
  {
    timesig ttest;
    try
    {
      ttest = tstamp[idx];
    }
    catch(exception &e)
    {
      return -1;
    }
    if(Ts.toSec() < ttest.toSec())
      idx++;
    else
      return idx;
  }

  return -1;
}
