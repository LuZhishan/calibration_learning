#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>        // 投影至平面
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>              // 提取边界
#include <pcl/common/impl/intersections.hpp>    // 计算直线的交点

using namespace cv;
using namespace std;


void camera_feature_extract(string img_file_name)
{

}

// 输入的文件名，存放提取出来的四个角点的点云
void lidar_feature_extract(string pcd_file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &corner_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_file_name, *source_cloud);

    // 过滤无关区域
    pcl::CropBox<pcl::PointXYZ> cb_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cb_filter.setInputCloud(source_cloud);
    cb_filter.setMin(Eigen::Vector4f(8.0, 1.2, -1.2, 1.0));// 四个参数分别是x_min, y_min, z_min, 1
    cb_filter.setMax(Eigen::Vector4f(8.5, 2.7,  0.6, 1.0));
    cb_filter.setNegative(false);
    cb_filter.filter(*filtered_cloud);

    // 提取平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.06);
    seg.setInputCloud(filtered_cloud);    
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // 投影至平面
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(plane_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected_cloud);

    // 提取法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(projected_cloud);
	ne.setRadiusSearch(0.1);
	ne.compute(*normals);
    // 提取边界
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
	be.setInputCloud(projected_cloud);
	be.setInputNormals(normals);
	be.setRadiusSearch(0.1);
	be.setAngleThreshold(M_PI / 4);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	be.setSearchMethod(tree);
	be.compute(boundaries);

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < projected_cloud->points.size(); i++)
	{
		if (boundaries[i].boundary_point == 0)
		{
			boundary_cloud->push_back(projected_cloud->points[i]);
		}
	}

    // 提取边线
    vector<pcl::ModelCoefficients> coefficients_line_vectors;       // 每条边的参数
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> left_edge_vectors;  // 去除部分边线后剩余的边线
    left_edge_vectors.push_back(boundary_cloud);

    pcl::SACSegmentation<pcl::PointXYZ> line_seg;               // 直线提取器
    pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);  // 直线的各项参数
    pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices); // 直线上所有点的索引值
    pcl::ExtractIndices<pcl::PointXYZ> extract_line;
    line_seg.setModelType(pcl::SACMODEL_LINE);
    line_seg.setMethodType(pcl::SAC_RANSAC);
    line_seg.setDistanceThreshold(0.05);
    for (size_t i = 0; i < 4; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        line_seg.setInputCloud(left_edge_vectors[i]);
        line_seg.segment(*inliers_line, *coefficients_line);
        coefficients_line_vectors.push_back(*coefficients_line); // 把当前边的参数存起来

        extract_line.setInputCloud(left_edge_vectors[i]);
        extract_line.setIndices(inliers_line);

        extract_line.setNegative(true);     // 把去除当前边后剩下的点存起来
        extract_line.filter(*temp_cloud);
        left_edge_vectors.push_back(temp_cloud);
    }

    Eigen::Vector4f point;
    pcl::PointXYZ pcl_point;

    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            Eigen::Vector3d line_vector1(coefficients_line_vectors[i].values[3], coefficients_line_vectors[i].values[4], coefficients_line_vectors[i].values[5]);
            Eigen::Vector3d line_vector2(coefficients_line_vectors[j].values[3], coefficients_line_vectors[j].values[4], coefficients_line_vectors[j].values[5]);
            if(fabs(line_vector1.dot(line_vector2)) < 0.9)
            {
                pcl::lineWithLineIntersection(coefficients_line_vectors[i], coefficients_line_vectors[j], point);
                pcl_point.x = point.x();
                pcl_point.y = point.y();
                pcl_point.z = point.z();
                corner_points->points.push_back(pcl_point);
            }
        }
    }

    // pcl::visualization::PCLVisualizer viewer("Title of Windows");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_of_plane(projected_cloud, 255, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_of_boundary(boundary_cloud, 0, 255, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_of_corner(corner_points, 255, 255, 255);
    // viewer.addPointCloud(corner_points, color_of_corner, "corner");
    // viewer.addPointCloud(projected_cloud, color_of_plane, "plane");
    // viewer.addPointCloud(boundary_cloud, color_of_boundary, "boundry");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "corner");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "boundry");
    // viewer.spin();
}






int main(int argc, char** argv)
{
    if(argc != 2)
    {
        cout << "Usage: ./" << endl;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corner_points(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_feature_extract(argv[1], lidar_corner_points);



    return 0;
}