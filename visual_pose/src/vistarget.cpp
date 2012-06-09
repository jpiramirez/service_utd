#include "vistarget.h"

vistarget::vistarget()
{
	cv::namedWindow("vistarget");
}

vistarget::~vistarget()
{
	cv::destroyWindow("vistarget");
}

void vistarget::computeMarkerLocations(Mat image)
{
	Mat intimg;
	int i, j;

	cvtColor(image, intimg, CV_BGR2GRAY);
	
	threshold(intimg, intimg, 128, 255, THRESH_BINARY_INV | THRESH_OTSU);
	cvtColor(intimg, internal, CV_GRAY2BGR);


	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(intimg.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	vector<vector<Point> >::iterator citer;
	vector<Moments> mset;

	for(i = 0; i < contours.size(); i++)
	{
		Moments tmpmomt;
		// Discarding the internal contours of the external
		// circles
		if(hierarchy[i].val[3] != -1)
			continue;
		tmpmomt  = moments(contours[i], false);
		// Filtering the blobs by size
		if (tmpmomt.m00 > 3)
			mset.push_back(tmpmomt);
	}

	vector<Point> markerMatches;

	cout << "---" << endl;
	for(i = 0; i < mset.size() - 1; i++)
		for(j = i + 1; j < mset.size(); j++)
		{
			if (blobDistance(mset[i], mset[j]) < 2)
			{
				cout << blobDistance(mset[i], mset[j]) << endl;
				markerMatches.push_back(Point(i, j));
				break;
			}
		}

	markers.clear();
	Point2f tmark;

	for(i = 0; i < markerMatches.size(); i++)
	{
		tmark.x = mset[markerMatches[i].x].m10/mset[markerMatches[i].x].m00;
		tmark.y = mset[markerMatches[i].x].m01/mset[markerMatches[i].x].m00;
		markers.push_back(tmark);
	}

	if(markers.size() != 5)
	{
		targetAcquired = false;
		return;
	}

	bool dupfound = false;
	i = 0;
	while(!dupfound && i < markers.size()-1)
	{
		for(j = i + 1; j < markers.size(); j++)
		{
			if(pointDistance(markers[i], markers[j]) < 2.0)
			{
				tmark = markers[0];
				markers[0] = markers[i];
				markers[i] = tmark;
				markers.erase(markers.begin() + j);
				dupfound = true;
				break;
			}
		}
		i++;
	}

	targetAcquired = true;
}

void vistarget::estimatePose()
{
	int i;
	if(!targetAcquired)
	{
		cout << "No target acquired." << endl;
		return;
	}

	Moments mmt;
	vector<Point> tgtcont;
	vector<Point> hull;
	for(i=0; i < 4; i++)
		tgtcont.push_back(Point(markers[i].x, markers[i].y));

	convexHull(tgtcont, hull, true, true);
	mmt = cv::moments(hull, false);
	
	Point ul, ll, ur, lr;

	i = 0;
	while(hull[i].x != tgtcont[0].x && hull[i].y != tgtcont[0].y)
		i++;

	ul = hull[i];
	ur = hull[(i+1)%4];
	lr = hull[(i+2)%4];
	ll = hull[(i+3)%4];

	Point2f center;
	center = Point2f(mmt.m10/mmt.m00, mmt.m01/mmt.m00);
	cout << center << endl;
	cv::circle(internal, Point(center.x, center.y), 7, CV_RGB(255,0,0));
	cv::line(internal, ul, ur, CV_RGB(255,0,0));
	cv::line(internal, ur, lr, CV_RGB(0,255,0));
	cv::line(internal, lr, ll, CV_RGB(0,0,255));
	cv::line(internal, ll, ul, CV_RGB(255,0,255));

	double f = 176;
	position.val[2] = f*tgtSize/sqrt(pow(ul.x-lr.x,2.0) + pow(ul.y-lr.y,2.0));
	position.val[0] = ((lr.x-ul.x)-88.0)/f;
	position.val[1] = ((lr.y-ul.y)-72.0)/f;

	cout << "Position: X=" << position.val[0] << " Y=" << position.val[1] << " Z=" << position.val[2] << endl;

	cv::imshow("vistarget", internal);

}

double vistarget::blobDistance(Moments a, Moments b)
{
	double mdist;
	Point2f pa, pb;

	pa.x = a.m10/a.m00;
	pa.y = a.m01/a.m00;
	pb.x = b.m10/b.m00;
	pb.y = b.m01/b.m00;

	mdist = sqrt(pow(pa.x-pb.x, 2.0) + pow(pa.y-pb.y, 2.0));

	return mdist;
}

double vistarget::pointDistance(Point2f a, Point2f b)
{
	return sqrt(pow(a.x-b.x, 2.0) + pow(a.y-b.y, 2.0));
}

