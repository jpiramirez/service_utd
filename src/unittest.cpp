#include "targetstateestimator.h"
#include "particlefilter.h"
#include "urbanmap.hpp"
#include "inh_particlefilter.hpp"

int main(int argc, char **argv)
{
    targetStateEstimator tse(10, 10, 1, 0, 0.1, 0.1);
    //particleFilter pf(50, 0.8, 0.2, Vec3f(2,2,0), Vec3f(-2,-2,0), 0.1);
    particleFilter *pf;
    urbanmap um;
    inhParticleFilter *ipf;
    ros::Time::init();

    Rect r;
    r.x = 2;
    r.width = 2;
    r.y = 2;
    r.height = 5;
    tse.updateGrid(r, false);
    Mat grid = tse.getGrid();
    cout << grid;
    r.x = 5;
    r.y = 5;
    tse.updateGrid(r, true);
    cout << grid << endl;
    tse.MLE();
    cout << tse.mean << endl;
    cout << tse.std << endl;

    um.mkSampleMap();
    um.loadMap("test.yml");

    pf = new particleFilter(100, 1.0, 0.0, um, 0.5);
    for(int k=0; k < 100; k++)
        pf->predict(um, 0.1);
    //pf.update(Point2f(-1,-1), Point2f(1,1), true);
//    pf->display();

    ipf = new inhParticleFilter(100, 1, 0, um, 0.5, 10);
    for(int i=0; i < 10; i++)
    {
        ipf->simplePredict(um, 0.1);
//        if(i%3 == 0)
//            ipf->simpleUpdate(um, Point2d(2.0,0.1), Point2f(0.0,-0.1), true);
    }

    ipf->processOOSM(um, Point2d(1.0, 0.1), Point2d(0.0,-0.1), true, ipf->tstamp[4]);

    cout << "IPF with memsize " << ipf->phist.size() << endl;
    ipf->displayHist(false);


    Mat vismap(400, 400, CV_8UC3);
    um.drawMap(vismap);
    pf->drawParticles(um, vismap);
    imwrite("mapita.png", vismap);
    for(int k=0; k < 10; k++)
        pf->predict(um, 0.1);
    vismap = Scalar(0);
    um.drawMap(vismap);
    pf->drawParticles(um, vismap);
    imwrite("mapita2.png", vismap);
    for(int k=0; k < 10; k++)
        pf->predict(um, 0.1);
    pf->update(um, Point2d(1.1,1.1), Point2d(-1.1,-1.1), true);
    vismap = Scalar(0);
    um.drawMap(vismap);
    pf->drawParticles(um, vismap);
    imwrite("mapita3.png", vismap);
//    pf->display();

    return 0;
}
