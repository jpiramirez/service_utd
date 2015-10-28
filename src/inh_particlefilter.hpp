#ifndef INH_PARTICLEFILTER_HPP
#define INH_PARTICLEFILTER_HPP

// Particle filter estimator with Inheritance
//
// Juan Pablo Ramirez <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Sensing, Robotics, Vision, Control and Estimation Lab
// (SeRViCE) 2012-2015


#include "particlefilter.h"
#include <stdexcept>

using namespace cv;
using namespace std;

typedef ros::Time timesig;

template <class T> class cyclicArray
{
public:
      int len;
      vector<T> store;
      int head, tail;

      cyclicArray()
      {
          len = 0;
          head = 0;
          tail = 0;
      }

      cyclicArray(int length)
      {
          len = length;
          if(length < 0)
              throw std::out_of_range("Negative cyclic array lengths are not allowed.");
          for(int i=0; i < len; len++)
          {
              T emptyElem;
              store.push_back(emptyElem);
          }
          head = 0;
          tail = len-1;
      }

      void prealloc(int length)
      {
          if(len > 0)
              throw std::logic_error("The cyclic array is already allocated.");
          len = length;
          if(length < 0)
              throw std::out_of_range("Negative cyclic array lengths are not allowed.");
          for(int i=0; i < len; i++)
          {
              T emptyElem;
              store.push_back(emptyElem);
          }
          head = 0;
          tail = len-1;
      }

      T getElement(int index)
      {
          if(index >= len)
              throw std::out_of_range("Index is larger than array length.");
          int idx = (head + index) % len;
          return store.at(idx);
      }

      T operator[](int index)
      {
          return getElement(index);
      }

      void putElement(T elem, int index)
      {
          if(index > len)
              throw std::out_of_range("Index is larger than array length.");
          int idx = (head + index) % len;
          store.at(idx) = elem;
      }

      // Push an element to the front of the array. This overwrites the last element.
      void pushElement(T elem)
      {
          store.at(tail) = elem;
          head--;
          tail--;
          if(head < 0)
              head = len-1;
          if(tail < 0)
              tail = len-1;
      }

      int size()
      {
          return len;
      }
};

class inhParticleFilter : public particleFilter
{
public:
    int memsize;
    cyclicArray<vector<Vec3f> > phist;
    cyclicArray<vector<double> > whist;
    cyclicArray<timesig> tstamp;
    cyclicArray<vector<int> > pid;
    int maxid;

    inhParticleFilter(int N, float alpha, float beta, urbanmap um, double maxvel, int memsize) : particleFilter(N, alpha, beta, um, maxvel)
    {
        if(memsize < 1)
        {
            throw std::logic_error("The PF memory cannot be smaller than one sample.");
        }
        this->memsize = memsize;
        phist.prealloc(memsize);
        whist.prealloc(memsize);
        tstamp.prealloc(memsize);
        pid.prealloc(memsize);
        vector<int> idarray;
        for(int i=0; i < N; i++)
            idarray.push_back(i);
        for(int i=0; i < memsize; i++)
        {
            phist.pushElement(pp);
            whist.pushElement(w);
            pid.pushElement(idarray);
            tstamp.pushElement(timesig::now());
        }
        maxid = N-1;
    }

    void update(urbanmap um, Point2d ul, Point2d br, bool measurement, int index)
    {
        this->pp = phist.getElement(index);
        this->w = whist[index];
        particleFilter::update(um, ul, br, measurement);
        phist.putElement(this->pp, index);
        whist.putElement(this->w, index);
    }

    void predict(urbanmap um, double Ts, int index)
    {
        this->pp = phist.getElement(index);
        particleFilter::predict(um, Ts);
        phist.putElement(this->pp, index);
    }

    void simplePredict(urbanmap um, double Ts)
    {
        this->pp = phist.getElement(0);
        this->w = whist.getElement(0);
        particleFilter::predict(um, Ts);
        phist.pushElement(this->pp);
        whist.pushElement(this->w);
        tstamp.pushElement(timesig::now());
    }

    void simpleUpdate(urbanmap um, Point2d ul, Point2d br, bool measurement);
    void processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, int index, double alpha, double beta);
    void processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, int index);
    void processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, timesig Ts);
    void processOOSM(urbanmap um, Point2d ul, Point2d br, bool measurement, timesig Ts, double alpha, double beta);
    int findIndex(timesig Ts);

    void displayHist(bool full)
    {
        timesig tnow = timesig::now();
        if(full)
        {
            for(int i=0; i < memsize; i++)
            {
                ros::Duration dur;
                dur = tnow-tstamp[i];
                std::cout << "Time: " << dur.toSec() << std::endl;
                for(int j=0; j < N; j++)
                    cout << "E: " << phist[i][j][0] << "  P: " << phist[i][j][1] << "  V: " << phist[i][j][2] << "  W: " << whist[i][j] << endl;
            }
            cout << endl << endl;
        }
        cout << "Inheritance" << endl;
        for(int i=0; i < memsize; i++)
        {
            for(int j=0; j < N; j++)
            {
                cout << pid[i][j] << " ";
            }
            cout << endl;
        }
    }
};



#endif // INH_ParticleFilter_HPP
