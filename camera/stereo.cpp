#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage: ./<progress_name> <left_image_path> <right_image_path>" << endl;
        return -1;
    }

    Mat img_l, img_r;
    Mat img_lg, img_rg;
    vector<String> image_path_l;            // 左摄像头图像的路径
           vector<Point2f>  corners_l;      // 左图检测到的角点
    vector<vector<Point2f>> all_points_l;   // 左图所有图片上的角点
    vector<String> image_path_r;            // 右摄像头图像的路径
           vector<Point2f>  corners_r;      // 右图检测到的角点
    vector<vector<Point2f>> all_points_r;   // 右图所有图片上的角点
    double square_size = 0.02423;           // 棋盘格的大小
           vector<Point3f>  chess_points;   // 每张图上的棋盘格点在世界坐标系下的坐标
    vector<vector<Point3f>> world_points;   // 所有图片上的棋盘格点在世界坐标系下的坐标
    for (size_t i = 0; i < 9; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            chess_points.push_back(Point3f(j*square_size, i*square_size, 0));   // 棋盘格上的点都有一个三维坐标
        }
        
    }

    glob(argv[1], image_path_l);
    glob(argv[2], image_path_r);
    for (size_t i = 0; i < image_path_l.size(); i++)
    {
        img_l = imread(image_path_l[i]);
        img_r = imread(image_path_r[i]);
        cvtColor(img_l, img_lg, COLOR_BGR2GRAY);
        cvtColor(img_r, img_rg, COLOR_BGR2GRAY);
        bool found_corners_l = findChessboardCorners(img_lg, Size(6, 9), corners_l);
        bool found_corners_r = findChessboardCorners(img_rg, Size(6, 9), corners_r);
        if (found_corners_l && found_corners_r)
        {
            TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.001);
            cornerSubPix(img_lg, corners_l, Size(5, 5), Size(-1, -1), criteria);
            cornerSubPix(img_rg, corners_r, Size(5, 5), Size(-1, -1), criteria);

            all_points_l.push_back(corners_l);
            all_points_r.push_back(corners_r);
            world_points.push_back(chess_points);
        }
        else
        {
            cout << "No corner points detect in left or right image" << endl;
        }
    }

    img_l = imread(image_path_l[0]);
    img_r = imread(image_path_r[0]);

    // 单独计算每个相机的内参
	Mat cameraMatrix_l, distCoeffs_l, cameraMatrix_r, distCoeffs_r;
    vector<Mat> R0, t0;
	calibrateCamera(world_points, all_points_l, img_l.size(), cameraMatrix_l, distCoeffs_l, R0, t0);
	calibrateCamera(world_points, all_points_r, img_r.size(), cameraMatrix_r, distCoeffs_r, R0, t0);
    // 计算两个相机的外参、本质矩阵、基础矩阵
    Mat R, t, E, F;
    stereoCalibrate(world_points, all_points_l, all_points_r, 
                    cameraMatrix_l, distCoeffs_l, cameraMatrix_r, distCoeffs_r, img_l.size(),R, t, E, F);
    // 计算立体校正的参数
    Mat R1, R2, P1, P2, Q;
    stereoRectify(cameraMatrix_l, distCoeffs_l, cameraMatrix_r, distCoeffs_r, img_l.size(), R, t, R1, R2, P1, P2, Q);

    Mat lmapx, lmapy, rmapx, rmapy;
    Mat img_lu, img_ru;
    initUndistortRectifyMap(cameraMatrix_l, distCoeffs_l, R1, P1, img_l.size(), CV_32F, lmapx, lmapy);
    initUndistortRectifyMap(cameraMatrix_r, distCoeffs_r, R2, P2, img_r.size(), CV_32F, rmapx, rmapy);
    remap(img_l, img_lu, lmapx, lmapy, cv::INTER_LINEAR);
    remap(img_r, img_ru, rmapx, rmapy, cv::INTER_LINEAR);

    imwrite("../l.png", img_lu);
    imwrite("../r.png", img_ru);
    // imshow("l", img_lu);
    // imshow("r", img_ru);
    // waitKey(0);

    return 0;
}