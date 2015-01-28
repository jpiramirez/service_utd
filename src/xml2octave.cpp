#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    string name(argv[1]);
    FileStorage fs(name, FileStorage::READ);
    Mat data;
    fs["G"] >> data;
    for(int i=0; i < data.rows; i++)
    {
      for(int j=0; j < data.cols; j++)
      {
	cout << data.at<float>(i,j) << " " ;
      }
      cout << endl;
    }
    return 0;
}
