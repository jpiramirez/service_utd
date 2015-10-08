#include "urbanmap.hpp"

urbanmap::urbanmap()
{

}

urbanmap::~urbanmap()
{

}

void urbanmap::mkSampleMap()
{
    FileStorage fs("test.yml", FileStorage::WRITE);
    Mat conn = Mat::zeros(Size(25,25), CV_32SC1);
    conn.at<int>(1,0) = 1;
    conn.at<int>(2,1) = 2;
    conn.at<int>(3,2) = 3;
    conn.at<int>(4,3) = 4;
    conn.at<int>(5,6) = 5;
    conn.at<int>(6,7) = 6;
    conn.at<int>(7,8) = 7;
    conn.at<int>(8,9) = 8;
    conn.at<int>(11,10) = 9;
    conn.at<int>(12,11) = 10;
    conn.at<int>(13,12) = 11;
    conn.at<int>(14,13) = 12;
    conn.at<int>(15,16) = 13;
    conn.at<int>(16,17) = 14;
    conn.at<int>(17,18) = 15;
    conn.at<int>(18,19) = 16;
    conn.at<int>(20,21) = 17;
    conn.at<int>(21,22) = 18;
    conn.at<int>(22,23) = 19;
    conn.at<int>(23,24) = 20;
    conn.at<int>(0,5) = 21;
    conn.at<int>(6,1) = 22;
    conn.at<int>(2,7) = 23;
    conn.at<int>(8,3) = 24;
    conn.at<int>(9,4) = 25;
    conn.at<int>(5,10) = 26;
    conn.at<int>(11,6) = 27;
    conn.at<int>(7,12) = 28;
    conn.at<int>(13,8) = 29;
    conn.at<int>(14,9) = 30;
    conn.at<int>(10,15) = 31;
    conn.at<int>(16,11) = 32;
    conn.at<int>(12,17) = 33;
    conn.at<int>(18,13) = 34;
    conn.at<int>(19,14) = 35;
    conn.at<int>(15,20) = 36;
    conn.at<int>(21,16) = 37;
    conn.at<int>(17,22) = 38;
    conn.at<int>(23,18) = 39;
    conn.at<int>(24,19) = 40;

    coord.clear();
    coord.push_back(Point2d(2,2));
    coord.push_back(Point2d(2,1));
    coord.push_back(Point2d(2,0));
    coord.push_back(Point2d(2,-1));
    coord.push_back(Point2d(2,-2));
    coord.push_back(Point2d(1,2));
    coord.push_back(Point2d(1,1));
    coord.push_back(Point2d(1,0));
    coord.push_back(Point2d(1,-1));
    coord.push_back(Point2d(1,-2));
    coord.push_back(Point2d(0,2));
    coord.push_back(Point2d(0,1));
    coord.push_back(Point2d(0,0));
    coord.push_back(Point2d(0,-1));
    coord.push_back(Point2d(0,-2));
    coord.push_back(Point2d(-1,2));
    coord.push_back(Point2d(-1,1));
    coord.push_back(Point2d(-1,0));
    coord.push_back(Point2d(-1,-1));
    coord.push_back(Point2d(-1,-2));
    coord.push_back(Point2d(-2,2));
    coord.push_back(Point2d(-2,1));
    coord.push_back(Point2d(-2,0));
    coord.push_back(Point2d(-2,-1));
    coord.push_back(Point2d(-2,-2));

    fs << "conn" << conn;
    this->conn = conn.clone();

    fs << "coord" << "[";
    for(int i=0; i < coord.size(); i++)
    {
        fs << "{";
        fs << "x" << coord[i].x << "y" << coord[i].y;
        fs << "}";
    }
    fs << "]";

    fs.release();
}

void urbanmap::loadMap(string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    fs["conn"] >> conn;
    FileNode coordinates = fs["coord"];
    FileNodeIterator it = coordinates.begin(), it_end = coordinates.end();

    coord.clear();
    Point2d pt;
    wayln.clear();
    double len;

    for( ; it != it_end; ++it)
    {
        pt.x = (double)(*it)["x"];
        pt.y = (double)(*it)["y"];
        coord.push_back(pt);
        cout << pt << endl;
    }

    int i, j;

    ne = coord[0];
    sw = coord[0];
    for(i=0; i < coord.size(); i++)
    {
        if(coord[i].x > ne.x)
            ne.x = coord[i].x;
        if(coord[i].y > ne.y)
            ne.y = coord[i].y;
        if(coord[i].x < sw.x)
            sw.x = coord[i].x;
        if(coord[i].y < sw.y)
            sw.y = coord[i].y;
    }
    cout << "Map Boundaries" << endl;
    cout << "NE: " << ne << "  SW: " << sw << endl;

    for(i=0; i < conlist.size(); i++)
        conlist[i].clear();

    double ned;
    minMaxIdx(conn, NULL, &ned);
    nedges = (int)ned;
    cout << "Map edges: " << ned << endl;

    for(i=0; i < nedges; i++)
    {
        wayln.push_back(0.0);
        elist.push_back(Point(0,0));
    }

    cout << "(";
    vector<int> endp;
    for(i=0; i < conn.rows; i++)
    {
        endp.clear();
        cout << "(";
        for(j=0; j < conn.cols; j++)
        {
            int u = conn.at<int>(i,j);
            if(u > 0)
            {
                endp.push_back(j);
                cout << (int)j << ",";
                len = pow(coord[i].x-coord[j].x,2.0) + \
                      pow(coord[i].y-coord[j].y,2.0);
                len = sqrt(len);
                wayln[u-1] = len;
                cout << "W" << u-1 << "  L" << len << ",";
                elist[u-1] = Point(i,j);
            }
        }
        conlist.push_back(endp);
        cout << ")";
    }
    cout << ")";

    fs.release();

}

Point urbanmap::xy2uv(int rows, int cols, Point2d q)
{
    double xlen = ne.x - sw.x;
    double ylen = ne.y - sw.y;
    Point uv;
    uv.x = cols-cols*(q.y-sw.y)/ylen;
    uv.y = rows-rows*(q.x-sw.x)/xlen;
    if(uv.x < 1)
        uv.x = 1;
    if(uv.y < 1)
        uv.y = 1;
    if(uv.x >= cols)
        uv.x = cols-1;
    if(uv.y >= rows)
        uv.y = rows-1;

    return uv;
}

void urbanmap::drawMap(Mat vismap)
{
    Point start, end;
    for(int i=0; i < elist.size(); i++)
    {
        start = xy2uv(vismap.rows, vismap.cols, coord[elist[i].x]);
        end = xy2uv(vismap.rows, vismap.cols, coord[elist[i].y]);
        line(vismap, start, end, CV_RGB(0,255,0));
    }
    for(int i=0; i < nedges; i++)
    {
        start = xy2uv(vismap.rows, vismap.cols, coord[i]);
        circle(vismap, start, 3, CV_RGB(255,0,0));
    }
}

bool urbanmap::isPointInEdge(const Point2d pt, double distance)
{

}


