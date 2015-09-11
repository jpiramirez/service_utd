#ifndef URBANMAP_HPP
#define URBANMAP_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

class urbanmap
{
public:
    Mat conn; //Connectivity matrix
    vector<Point2d> coord; //Node coordinates
    Point2d ne, sw; //Map boundaries
    vector<vector<int> > conlist;
    vector<double> wayln; //Road portion lengths
    vector<Point> elist; //Edge list (Initial and final nodes) to speed up
                         //lookups
    int nedges;
    urbanmap();
    ~urbanmap();
    bool isPointInEdge(const Point2d pt, double distance);
    void loadMap(string filename);
    void mkSampleMap();
    void drawMap(Mat vismap);
    Point xy2uv(int rows, int cols, Point2d q);

    Point2d ep2coord(int edge, double pos)
    {
        Point2d pt;
        Point2d start = coord[elist[edge].x];
        Point2d end = coord[elist[edge].y];
        pt.x = start.x + pos*(end.x-start.x);
        pt.y = start.y + pos*(end.y-start.y);

        return pt;
    }

};

#endif // URBANMAP_HPP
