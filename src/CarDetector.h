#ifndef CARDETECT_H
#define CARDETECT_H

#include <highgui.h>
#include <cv.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


////////////////////////////////////////////////

//--------------./NAME image image2 Cascade.xml Cascade2.xml Cascade3.xml

//------------image  	image to detect car
//------------image2 	image to get color
//------------Cascade   file *.xml
////////////////////////////////////////////////


using namespace std;
using namespace cv;

RNG rng(12345);


//-------------------------------------------------------Prototypes
void GetRectangle(vector<Rect> &rect, vector<double> &AreaBlob, CvSeq *contours);
void DrawRectangle(Mat img, vector<Rect> &rect);
int DrawBiggestRectangle(Mat img, vector<Rect> &rect, vector<double> &Area);
void Dilation(Mat img, Mat dest, int erosion_size);
//----------------------------------------------------------------------Orientation
int CountWhitePixel(Mat I);
void HistStretching(Mat I, Mat O, int Gmax, int Gmin);
//-----------------------------------------------------------------------------Color
void PickUpColor(Mat I, int x, int y, Scalar &q);
void EuclidianInRange(Mat img, Mat O, Scalar &q, int range);


class CarDetector
{
	private:
		String car_cascade_name[3];
		CascadeClassifier car_cascade[3];
		Scalar Color;
		vector<double> AreaClassifier;
		vector<Rect> cars[3];		
		int x;
		int y;
		int width;
		int height;
		void FindCars(Mat frame);
		int CarsOrientation(Mat Src);
		void ThresholdClassifier(Mat img, int argum);
				
	public:
		CarDetector(String CascadeVertical, String CascadeHorizontal, String CascadeDiagonal);
		~CarDetector();
        bool DetectCar(Mat frame, Scalar Color, int RatioColor);
		int GetX();
		int GetY();
		int GetWIDTH();
		int GetHEIGHT();
};

//----------------------------------------------------------------------------------------------------------constructor
CarDetector::CarDetector(String CascadeHorizontal, String CascadeDiagonal, String CascadeVertical)
{
	int i;	
	printf("\nRemember put the cascades in the next order: Horizontal Diagonal Vertical\n");
	cvWaitKey(2000);
	car_cascade_name[0]=CascadeHorizontal;
	car_cascade_name[1]=CascadeDiagonal;
	car_cascade_name[2]=CascadeVertical;
	x=0;
	y=0;
	width=0;
	height=0;
	//-----------------Load the cascade
	for(i=0;i<3;i++)
	{	
		if( !car_cascade[i].load( car_cascade_name[i] ) )
		{ 
			printf("\n\n--(!)Error loading *.xml\n\n"); 
			exit(1); 
		};
	}
}



CarDetector::~CarDetector()
{

}



//-----------------------------------------------------------------------------------------------methods class CarDetector
bool CarDetector::DetectCar(Mat frame, Scalar Color, int RatioColor)
{	
	int index;
	CvMemStorage *storage = cvCreateMemStorage(0);
	Mat Lab, frame2;
	Mat C1,C2,C3,C5,C4,C6;
	vector<double> AreaBlob, AreaTotal;
	vector<Rect> rect, rect2;
	CvSeq *contours = 0;	
	this->Color=Color;
    bool detected = false;


	//---------------------------------------------------------Find cars in image
	frame2 = frame.clone();
	FindCars(frame2);
	C1 = Mat::zeros(frame.size(), CV_8UC1);
	ThresholdClassifier(C1, 3);  //3 defeault
	//imshow("C1", C1);
	
	//---------------------------------------------------------Range of Color
	cvtColor(frame, Lab, CV_BGR2Lab);//CV_RGB2Lab
	C2 = Mat::zeros(frame.size(), CV_8UC1);	
	EuclidianInRange(Lab, C2, Color, RatioColor);
	//imshow("C2", C2);
	
	//--------------------------------------------------------Find Contours Blob Color
	IplImage *C2A = new IplImage(C2);
	cvFindContours(C2A, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL);
	GetRectangle(rect, AreaBlob, contours);
	C3 = Mat::zeros(frame.size(), CV_8UC1);	
	DrawRectangle(C3, rect);
	//imshow("R L+a+b",C3);

	//--------------------------------------------------------Classifier AND Blob
	C4 = Mat::zeros(frame.size(), CV_8UC1);	
	bitwise_and(C1,C3,C4);	
	//imshow("LAB+C5",C4);
	C5 = Mat::zeros(frame.size(), CV_8UC1);	
	Dilation( C4, C5, 5);
	//imshow("erosion",C5);

	//--------------------------------------------------------Find Contours Biggest Area
	IplImage *C5A = new IplImage(C5);
	cvFindContours(C5A, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL);
	GetRectangle(rect2, AreaTotal, contours);
	//C6 = Mat::zeros(frame.size(), CV_8UC1);	
	//index=DrawBiggestRectangle(C6, rect2, AreaTotal);
	//imshow("F",C6);	
	index=DrawBiggestRectangle(frame, rect2, AreaTotal);
//    imwrite("delme.png", frame);
	if(index>=0)
	{
		x=rect2[index].x;
		y=rect2[index].y;
		width=rect2[index].width;
		height=rect2[index].height;
        detected = true;
	}

    return detected;
}



void CarDetector::FindCars(Mat frame)
{
	Mat frame_gray;
	int i=0, orientation;
	
	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	//equalizeHist(frame_gray, frame_gray);
	AreaClassifier.clear();
	orientation=CarsOrientation(frame_gray);
	//-- Detect cars	
	car_cascade[orientation].detectMultiScale(frame_gray, cars[orientation], 1.1, 20, 0, Size(20,20));
	printf("\n*********CLassifier************\n");
	printf("cars detected=%lu\n",cars[orientation].size());
	for( i = 0; i < cars[orientation].size(); i++ )
	{
		if(cars[orientation].size()==0) break;
		Point p1(cars[orientation][i].x, cars[orientation][i].y);
		Point p2(cars[orientation][i].x+cars[orientation][i].width , cars[orientation][i].y+cars[orientation][i].height);
		rectangle(frame, p1, p2, Scalar( 255, 0, 0 ), 1, 8, 0 );
		AreaClassifier.push_back(cars[orientation][i].width*cars[orientation][i].height); 
		// printf("AreaClassifier[%d] = %f \n",i,AreaClassifier[i]);
	}	

	//-- Show image
	imshow("Car_detection", frame);
}



int CarDetector::CarsOrientation(Mat Src)  // 0=Horinzontal, 1=Dioagonal, 2=Vertical   
{
	Mat Blur, GX, AGX, GY, AGY, OtsuX, OtsuY; 
	int Nx=0, Ny=0, orientation;
	float k=1.45;
	
	blur(Src, Blur, Size(2,2));
	//imshow("Blur", Blur);
	
	//------X	
	Sobel(Blur, GX, CV_16S, 1, 0, 1);
 	convertScaleAbs(GX, AGX);
	GX = Mat::zeros(AGX.size(),AGX.type());
	HistStretching(AGX, GX, 255, 0);
	threshold(GX, OtsuX, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//imshow("GX", OtsuX);
	Nx=CountWhitePixel(OtsuX);	
	
	//-------Y
	Sobel(Blur, GY, CV_16S, 0, 1, 1);
 	convertScaleAbs(GY, AGY);
	GY = Mat::zeros(AGY.size(),AGY.type());	
	HistStretching(AGY, GY, 255, 0);
	threshold( GY, OtsuY, 0, 255, THRESH_BINARY | THRESH_OTSU);	
	//imshow("GY", OtsuY);
	Ny=CountWhitePixel(OtsuY);		
	
	// printf("Count X = %d Count Y = %d   X/Y=%f \n",Nx,Ny,(float)Nx/(float)Ny);
	if((float)Nx/(float)Ny > k) 
	{
		printf("Using Vertical Cars Detector\n");
		orientation=2;
	}
	if((float)Ny/(float)Nx > k) 
	{	
		printf("Using Horizontal Cars Detector\n");
		orientation=0;
	}
	if( (float)Nx/(float)Ny < k && (float)Ny/(float)Nx < k) 
	{
		printf("Using Diagonal Cars Detector\n");
		orientation=1;
	}
	return orientation;
}



void CarDetector::ThresholdClassifier(Mat img, int argum)
{
	int i=0,j=0;
	for(j=0;j<argum;j++)
	{
		//printf("CS=%lu\n",cars[j].size());
		for( size_t i = 0; i < cars[j].size(); i++ )
		{
			if(cars[j].size()==0) break;
			Point p1( cars[j][i].x, cars[j][i].y );
			Point p2( cars[j][i].x+cars[j][i].width , cars[j][i].y+cars[j][i].height );
			rectangle(img, p1, p2, Scalar( 255, 0, 0 ), CV_FILLED, 8, 0 );
		}
	}		
}



int CarDetector::GetX()
{
	return x;
}



int CarDetector::GetY()
{
	return y;
}



int CarDetector::GetWIDTH()
{
	return width;
}



int CarDetector::GetHEIGHT()
{
	return height;
}







//----------------------------------------------------------------------------------------------------------Functions

void GetRectangle(vector<Rect> &rect, vector<double> &AreaBlob, CvSeq *contours)
{
	int i=0, j=0;

	rect.clear();
	AreaBlob.clear();
	for(; contours!=0; contours = contours->h_next)
	{
		rect.push_back(cvBoundingRect(contours, 0));
		if( ((float)rect[j].width/(float)rect[j].height) > 2.35 ||  ((float)rect[j].width/(float)rect[j].height) < 0.35) 
		{
			rect.pop_back();
			continue;
		}
		else
		{
			//--------------Small area
			if(rect[j].width*rect[j].height < 100) 
			{	
				rect.pop_back();
				continue;
			}
			AreaBlob.push_back(rect[j].width*rect[j].height); 
			j++;			
		} 	  
	}
	//printf("Rectangles = %d\n",j);
}



void DrawRectangle(Mat img, vector<Rect> &rect)
{
	int i=0;
	//printf("size=%lu\n",rect.size());
	for(i=0; i<rect.size(); i++)
	{
		if( ((float)rect[i].width/(float)rect[i].height) > 2.35 ||  ((float)rect[i].width/(float)rect[i].height) < 0.35) 
		{
			continue;
		}
		else
		{
			//--------------Small area
			if(rect[i].width*rect[i].height < 100) 
			{
				continue;
			}
			rectangle(img,cvPoint(rect[i].x, rect[i].y),    
			cvPoint(rect[i].x+rect[i].width, rect[i].y+rect[i].height), cvScalar(255, 255, 0, 0), CV_FILLED, 8, 0);	
		}	         
	}
	//printf("Rectangles = %d\n",i);
}



int DrawBiggestRectangle(Mat img, vector<Rect> &rect, vector<double> &Area)
{
	int i=0, j=0;
	double AreaBiggest=0;

	//printf("Bsize=%lu\n",rect.size());
	//printf("BAreasize=%lu\n",Area.size());
	for(i=0; i<rect.size(); i++)
	{
		if( ((float)rect[i].width/(float)rect[i].height) > 2.35 ||  ((float)rect[i].width/(float)rect[i].height) < 0.35) 
		{
			continue;
		}
		else
		{
			//--------------Small area
			if(rect[i].width*rect[i].height < 100) 
			{
				continue;
			}	
		}	         
	}
	
	//------------Calculate the biggest area 
	j=0;
	for(i=0; i<Area.size(); i++)
	{	
		//printf("BArea[%d]=%f \n",i,Area[i]);
		if(Area[i]>AreaBiggest)
		{
			AreaBiggest=Area[i];
			j=i;
		}
	}
	if(rect.size()>0)
	{
		rectangle(img,cvPoint(rect[j].x, rect[j].y),    
		cvPoint(rect[j].x+rect[j].width, rect[j].y+rect[j].height), cvScalar(255, 255, 0, 0), 1, 8, 0);
		return j;
	}
	else
		return -1;
}



void Dilation(Mat img, Mat dest, int erosion_size)
{
	//-----MORPH_RECT  MORPH__CROSS  MORPH__ELLIPSE 

	 Mat element = getStructuringElement(MORPH_CROSS, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point	(erosion_size, erosion_size));
	dilate(img, dest, element);
}



//------------------------------------------------------------------------------------------------Orientation
int CountWhitePixel(Mat I)
{
	int i=0, j=0, count=0;	

	for(i=0;i<I.rows;i++)
	{
		for(j=0;j<I.cols;j++)
		{
			if(I.at<uchar>(i,j)==255) count++;	
		}		
	}


	
	return count;
}



void HistStretching(Mat I, Mat O, int Gmax, int Gmin)
{
	double min, max;
	int i,j;

	minMaxLoc(I, &min, &max);
	for(i=0;i<I.rows;i++)
	{
		for(j=0;j<I.cols;j++)
		{
			O.at<uchar>(i,j)=(Gmax-Gmin)*( ((I.at<uchar>(i,j))-min)/(max-min) ) + Gmin;	
		}		
	}
}




//------------------------------------------------------------------------------------------------------------------Color
void PickUpColor(Mat I, int x, int y, Scalar &q)
{
	int i, j, size=5, media[3]={0,0,0};
	
	IplImage *img = new IplImage(I);
	q=cvGet2D(img,x,y);
	//printf("q1=%f q2=%f q3=%f \n",q.val[0],q.val[1],q.val[2]);		
	for(i=(x-size/2);i<(x+size/2);i++)
	{
		for(j=(y-size/2);j<(y+size/2);j++)
		{
			if(i>0 && i<img->height && j>0 && j<img->width)
			{		
				q=cvGet2D(img,i,j);
				media[0] += q.val[0];
				media[1] += q.val[1];
				media[2] += q.val[2];
			}	
		}
	}
	q.val[0] = media[0]/((size/2)*2*(size/2)*2);
	q.val[1] = media[1]/((size/2)*2*(size/2)*2);
	q.val[2] = media[2]/((size/2)*2*(size/2)*2);
	//printf("Color: q1=%f q2=%f q3=%f \n",q.val[0],q.val[1],q.val[2]);		
}



void EuclidianInRange(Mat I, Mat O, Scalar &q, int range)
{
	int i,j;
	int cn=I.channels();
	float D;
	Scalar_<uint8_t> p;
	// printf("Rows=%d,Cols=%d\n",I.rows,I.cols);
	for(i=0;i<I.rows;i++)
	{
		for(j=0;j<I.cols;j++)
		{
			Vec3b p = I.at<Vec3b>(Point(j,i));
			D=pow((q.val[0]-p.val[0]),2)+pow((q.val[1]-p.val[1]),2)+pow((q.val[2]-p.val[2]),2);
			D=sqrt(D);
			if(D<range)
			{
				O.at<uchar>(Point(j,i)) = 255;
				
			}									
			else
			{
				O.at<uchar>(Point(j,i)) = 0;
			}			
		}
	}
} 


#endif
