#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "Usage: ./<progress_name> <image_path>" << endl;
        return -1;
    }

    Mat img_in, img_gray, img_out;
    double square_size = 0.02423;

    vector<String> image_path;      //创建容器存放读取图像路径
           vector<Point2f>  corner_points;  //每张图检测到的角点
    vector<vector<Point2f>> images_points;  //所有图片上的角点
           vector<Point3f>  chess_points;   //每张图上的棋盘格点在世界坐标系下的坐标
    vector<vector<Point3f>> world_points;   //所有图片上的棋盘格点在世界坐标系下的坐标


    for (size_t i = 0; i < 9; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            chess_points.push_back(Point3f(j*square_size, i*square_size, 0));   // 棋盘格上的点都有一个三维坐标
        }
        
    }
    
    glob(argv[1], image_path);
    for (size_t i = 0; i < image_path.size(); i++)
    {
        img_in = imread(image_path[i]);
        cvtColor(img_in, img_gray, COLOR_BGR2GRAY);
        bool found_corners = findChessboardCorners(img_gray, Size(6, 9),corner_points);  // 棋盘格6x9
        if (found_corners)
        {
            TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.001);  //迭代终止条件
            cornerSubPix(img_gray, corner_points, Size(11, 11), Size(-1, -1), criteria);//提取亚像素角点
            drawChessboardCorners(img_in, Size(6, 9), corner_points, found_corners);    //绘制角点
            images_points.push_back(corner_points);
            world_points.push_back(chess_points);
        }
        else
        {
            cout << "No corner points detect in this image" << endl;
        }

        // imshow("Draw Corners", img_in);
        // waitKey(100);
    }

	Mat cameraMatrix, distCoeffs;//内参矩阵，畸变系数，旋转量，偏移量
    vector<Mat> R, t;//旋转量，偏移量
	calibrateCamera(world_points, images_points, img_gray.size(), cameraMatrix, distCoeffs, R, t);

	cout << "cameraMatrix:" << endl << cameraMatrix << endl;
	cout << "*****************************" << endl;
	cout << "distCoeffs:"   << endl << distCoeffs << endl;
	cout << "*****************************" << endl;
	// cout << "R vectors:" << endl << R << endl;
    // cout << "t vectors:" << endl << t << endl;

    //计算重投影误差
    double e, e_sum = 0;
    int n, n_sum = 0;
    vector<Point2f> projected_points;
    for (size_t i = 0; i < world_points.size(); i++)
    {
        projectPoints(world_points[i], R[i], t[i], cameraMatrix, distCoeffs, projected_points);
        e = norm(Mat(projected_points) - Mat(images_points[i]), NORM_L2);
        e_sum = e_sum + e*e;
        n = world_points[i].size();
        n_sum = n_sum + n;
    }
    cout << "ReProjectError: " << sqrt(e_sum / n_sum) << endl;


    //将内参写入文件
    FileStorage fs("../config/right_cameraMatrix.yaml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs.release();

    //畸变矫正
    img_in = imread(image_path[0]);
    undistort(img_in, img_out, cameraMatrix, distCoeffs);
    imshow("undistort image", img_out);
    waitKey(0);

    return 0;
}
