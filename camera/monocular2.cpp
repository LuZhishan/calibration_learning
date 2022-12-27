#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) 
{
    if (argc != 2)
    {
        cout << "Usage: ./<progress_name> <left_image>" << endl;
        return -1;
    }
    Mat img = imread(argv[1]);
    if (img.data == NULL)
    {
        cout << "No such image" << endl;
    }

    Mat img_g;
    cvtColor(img, img_g, COLOR_BGR2GRAY);
    
	// Blob算子参数
	SimpleBlobDetector::Params params;
	params.minThreshold = 50;       //斑点二值化阈值
	params.maxThreshold = 200;
    // params.filterByCircularity = true;  //斑点圆度的限制变量，默认是不限制  
    // params.minCircularity = 0.8f;   //斑点的最小圆度  
    // params.maxCircularity = std::numeric_limits<float>::max(); 
    params.filterByColor = true;    //斑点颜色的限制变量  
    params.blobColor = 0;           //表示只提取黑色斑点；如果该变量为255，表示只提取白色斑点  
	params.filterByArea = true;     //斑点面积的限制
	params.maxArea = 10e4;          //斑点的最大面积
	params.minArea = 10;            //斑点的最小面积
	params.minDistBetweenBlobs = 5; //最小的斑点距离
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

	vector<Point2f> centers;
	Size patternSize(9, 6);	
	
	// 提取圆点特征的圆心
	bool found = findCirclesGrid(img_g, patternSize, centers, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);
	drawChessboardCorners(img, patternSize, centers, found);

	double sf = 960. / MAX(img.rows, img.cols);
	resize(img, img, Size(), sf, sf, INTER_LINEAR_EXACT);

	imshow("corners", img);

	waitKey();
	return 0;
}
