#include "particlefilter.h"

particleFilter::particleFilter(int N, float alpha, float beta, Vec3f upperbnd, Vec3f lowerbnd, double stddev)
{
    this->N = N;
    this->upperbnd = upperbnd;
    this->lowerbnd = lowerbnd;
    this->stddev = stddev;
    this->alpha = alpha;
    this->beta = beta;

    Vec3f np;

    T = gsl_rng_mt19937;
    r = gsl_rng_alloc(T);
    gsl_rng_env_setup();

    // Assuming no prior for p(x)
    for(int i=0; i < N; i++)
    {
        for(int k=0; k < 3; k++)
            np[k] = (upperbnd[k]-lowerbnd[k])*((double)rand()/(double)RAND_MAX)+lowerbnd[k];
        pp.push_back(np);
        w.push_back(1.0/(double)N);
    }

    pfType = TWODIM;
}

particleFilter::particleFilter(int N, float alpha, float beta, urbanmap um, double maxvel)
{
    this->N = N;
    this->alpha = alpha;
    this->beta = beta;

    gsl_rng_env_setup();
    T = gsl_rng_mt19937;
    r = gsl_rng_alloc(T);


    Vec3f np;
    pp.clear();
    this->maxvel = maxvel;

    double totalLen = 0.0;
    for(int i=0; i < um.wayln.size(); i++)
        totalLen += um.wayln[i];

    int pcount = 0;
    for(int i=0; i < um.wayln.size(); i++)
    {
        int partPerEdge = (int)((double)N*um.wayln[i]/totalLen);
        for(int j=0; j < partPerEdge; j++)
        {
            np[0] = i;
            pp.push_back(np);
            pcount++;
        }
    }

    if(pcount < N)
    {
        for(int i=pcount; i < N; i++)
        {
            np[0] = gsl_rng_uniform_int(r, um.nedges);
            pp.push_back(np);
        }
    }

    for(int i=0; i < N; i++)
    {
        pp[i][1] = gsl_rng_uniform(r);
        pp[i][2] = maxvel*gsl_rng_uniform(r);
        w.push_back(1.0/(double)N);
    }
    pfType = GRAPH;
}

void particleFilter::reset(urbanmap um)
{
    Vec3f np;
    pp.clear();
    w.clear();


    double totalLen = 0.0;
    for(int i=0; i < um.wayln.size(); i++)
        totalLen += um.wayln[i];

    int pcount = 0;
    for(int i=0; i < um.wayln.size(); i++)
    {
        int partPerEdge = (int)((double)N*um.wayln[i]/totalLen);
        for(int j=0; j < partPerEdge; j++)
        {
            np[0] = i;
            pp.push_back(np);
            pcount++;
        }
    }

    if(pcount < N)
    {
        for(int i=pcount; i < N; i++)
        {
            np[0] = gsl_rng_uniform_int(r, um.nedges);
            pp.push_back(np);
        }
    }

    for(int i=0; i < N; i++)
    {
        pp[i][1] = gsl_rng_uniform(r);
        pp[i][2] = maxvel*gsl_rng_uniform(r);
        w.push_back(1.0/(double)N);
    }
}

particleFilter::~particleFilter()
{
    gsl_rng_free(r);
}

// The motion model goes here
void particleFilter::predict()
{
    for(int i=0; i < N; i++)
    {
        pp[i][0] += gsl_ran_gaussian(r, stddev);
        while(pp[i][0] > upperbnd[0] || pp[i][1] > upperbnd[1] || pp[i][2] > upperbnd[2] || \
           pp[i][0] < lowerbnd[0] || pp[i][1] < lowerbnd[1] || pp[i][2] < lowerbnd[2])
        {
            pp[i][0] += gsl_ran_gaussian(r, stddev);
            pp[i][1] += gsl_ran_gaussian(r, stddev);
        }

    }
}

void particleFilter::predict(urbanmap um, double Ts)
{
    for(int i=0; i < N; i++)
    {
        //cout << "i:" << i << " LEN :" << (int)pp[i][0] << endl;
        pp[i][1] = pp[i][1]*um.wayln[(int)pp[i][0]] + Ts*pp[i][2];
        pp[i][1] /= um.wayln[(int)pp[i][0]];
        pp[i][2] = 3*maxvel*(gsl_rng_uniform(r)-2.0/3.0);
        if(pp[i][2] > maxvel)
            pp[i][2] = maxvel;
        if(pp[i][2] < 0)
            pp[i][2] = 0.0;
        if(pp[i][1] < 0.0)
            pp[i][1] = 0.0;
        //In case the relative position is beyond the end of the road
        if(pp[i][1] > 1.0)
        {
            int endnode = um.elist[(int)pp[i][0]].y;
            //Pick a way at random.
            int nways = um.conlist[endnode].size();
            if(nways > 0)
            {
                //Pick a random destination index
                int randway = gsl_rng_uniform_int (r, nways);
                //Grab the destination node with that index
                int ndestnode = um.conlist[endnode][randway];
                //Find the edge number corresponding to the departure and
                //arrival nodes
                pp[i][0] = um.conn.at<int>(endnode,ndestnode) - 1;
                //pp[i][1] -= 1.0; //incorrect, leave for the moment, come back to fix it
                pp[i][1] = 0.0;
            }
            else
            {
                pp[i][1] = 1.0;
            }
        }
    }
}

void particleFilter::display()
{
    if(pfType == TWODIM)
    {
        for(int i=0; i < N; i++)
        {
            cout << "X: " << pp[i][0] << "  Y: " << pp[i][1] << "  Z: " << pp[i][2] << "  W: " << w[i] << endl;
        }
    }
    else
    {
        for(int i=0; i < N; i++)
            cout << "E: " << pp[i][0] << "  P: " << pp[i][1] << "  V: " << pp[i][2] << "  W: " << w[i] << endl;

    }
}

void particleFilter::update(Point2f ul, Point2f br, bool measurement)
{
    double wsum = 0.0;
    for(int i=0; i < N; i++)
    {
        if(measurement)
        {
            if(pp[i][0] > ul.x && pp[i][1] > ul.y && pp[i][0] < br.x && pp[i][1] < br.y)
                w[i] *= alpha;
            else
                w[i] *= beta;
        }
        else
        {
            if(pp[i][0] > ul.x && pp[i][1] > ul.y && pp[i][0] < br.x && pp[i][1] < br.y)
                w[i] *= 1.0-alpha;
            else
                w[i] *= 1.0-beta;
        }
        wsum += w[i];
    }

    for(int i=0; i < N; i++)
        w[i] /= wsum;
}

void particleFilter::update(urbanmap um, Point2d ul, Point2d br, bool measurement)
{
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

    if(wsum < 0.001)
    {
        reset(um);
        return;
    }

    double Neff = 0.0;
    for(int i=0; i < N; i++)
    {
        w[i] /= wsum;
        Neff += pow(w[i], 2.0);
    }

    Neff = 1.0/Neff;
    if(isnan(Neff))
        Neff = 0.0;
    double Ns = N/100;
    if(Neff < Ns)
    {
//       cout << "Neff=" << Neff << " Ns=" << Ns << endl;
       this->resample();
    }
}

// In case the target has been detected and positional information is available,
// use this function to update the particles.
void particleFilter::update(urbanmap um, Point2d pos, double stddev)
{
    Point2d pt;
    double nw;
    double wsum = 0.0;
    for(int i=0; i < N; i++)
    {
        pt = um.ep2coord((int)pp[i][0], pp[i][1]);
        nw = gsl_ran_gaussian_pdf(pt.x-pos.x, stddev);
        nw *= gsl_ran_gaussian_pdf(pt.y-pos.y, stddev);
        w[i] *= nw;
        wsum += w[i];
    }

    if(wsum < 1e-100)
    {
        reset(um);
        return;
    }

    double Neff = 0.0;
    for(int i=0; i < N; i++)
    {
        w[i] /= wsum;
        Neff += pow(w[i], 2.0);
    }
    Neff = 1.0/Neff;
    if(isnan(Neff))
        Neff = 0.0;
    double Ns = (double)N/2.0;
    if(Neff < Ns)
    {
       this->resample();
    }
}

void particleFilter::update(Point2f mean, Mat &cov, bool measurement)
{

}

void particleFilter::resample()
{
    vector<double> u, wc;
    vector<int> ind;

    cout << "Resampling requested" << endl;

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
        npp[i] = pp[ind[i]];
        w[i] = 1.0/(double)N;
    }

    pp = npp;

}

void particleFilter::drawParticles(urbanmap um, Mat map)
{
    double meanw = 0.0;
    for(int i=0; i < N; i++)
    {
        if(isnan(w[i]))
        {
            cout << "NAN WEIGHT!!!!!" << endl;
            reset(um);
            return;
        }
        meanw += w[i];
    }
    meanw /= (double)N;
    for(int i=0; i < N; i++)
    {
        Point2d pt = um.ep2coord((int)pp[i][0], pp[i][1]);
        Point uv = um.xy2uv(map.rows, map.cols, pt);
        if(w[i] >= meanw)
            circle(map, uv, 3, CV_RGB(255,0,255));
        else
            circle(map, uv, 3, CV_RGB(0,255,255));
    }
}

Vec3f particleFilter::meanEstim(urbanmap um)
{
    float *partInEdge = new float[um.nedges];
    memset(partInEdge, 0, sizeof(float)*um.nedges);

    //Determine the edge with the most prob. weight
    for(int i=0; i < N; i++)
    {
        partInEdge[(int)pp[i][0]] += w[i];
    }

    float maxpart = 0;
    int domEdge = 0;
    for(int i=0; i < um.nedges; i++)
    {
        if(partInEdge[i] > maxpart)
        {
            maxpart = partInEdge[i];
            domEdge = i;
        }
    }


    //Picking the subset of particles that belong to that
    //"dominant" edge

    vector<Vec3f> subpart;
    vector<double> subw;
    double wsum = 0.0;
    for(int i=0; i < N; i++)
    {
        if((int)pp[i][0] == domEdge)
        {
            subpart.push_back(pp[i]);
            subw.push_back(w[i]);
            wsum += w[i];
        }
    }

    //Generating an estimate
    Vec3f mpart(domEdge,0,0);
    for(int i=0; i < subpart.size(); i++)
    {
        subw[i] /= wsum;
        mpart[1] += subpart[i][1]*subw[i];
        mpart[2] += subpart[i][2]*subw[i];
    }

    delete partInEdge;

    return mpart;
}
