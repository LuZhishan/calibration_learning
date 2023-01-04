#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main()
{
    Mat image(480 ,640, CV_8UC3, Scalar(0,0,0));
    vector<Point2f> points;
    for (size_t i = 0; i < 101; i+=10)
    {
        Point2f p1(i+300, i+50);
        points.push_back(p1);
        Point2f p2(i+200, i+150);
        points.push_back(p2);
        Point2f p3(i+200, 150-i);
        points.push_back(p3);
        Point2f p4(i+300, 250-i);
        points.push_back(p4);
    }

    
    for (size_t i = 0; i < points.size(); i++)
    {
        circle(image, points[i], 3, Scalar(0,0,255), -1);
    }
    
    imshow("0", image);
    waitKey(0);    

}