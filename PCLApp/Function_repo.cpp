#include<iostream>
#include "stdafx.h"
#include<fstream>
#include<vector>
#include<string>
#include<pcl\io\pcd_io.h>
#include<pcl\point_types.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\visualization\cloud_viewer.h>
#include<pcl\filters\approximate_voxel_grid.h>
#include<pcl\registration\ndt.h>
#include<boost/thread/thread.hpp>
#include <vtkPLYReader.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>
#include <vtkMassProperties.h>
#include <math.h>
using namespace std;

int multiviewerndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
int findplane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//transform_1(2, 0) =-0.135176; transform_1(2, 1) = 0.0436936; transform_1(2, 2) = 0.985858; transform_1(2, 3) = 38.0787;
// triangle2  -0.0526151  0.0518616   0.997267    36.0612
float A= -0.0526151;
float B= 0.0518616;
float C= 0.997267;
float D= 36.0612;

class Volume
{
public:
	float A = 0.0;
	float B = 0.0;
	float C = 0.0;
	float D = 0.0;
	double CalcualteV(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	double CalcualteV2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	const int nx = 1000;
	const int ny = 1000;
private:

};
double Volume::CalcualteV(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//transform_1(2, 0) =-0.135176; transform_1(2, 1) = 0.0436936; transform_1(2, 2) = 0.985858; transform_1(2, 3) = 38.0787;
	transform_1(2, 0) = A;
	transform_1(2, 1) = B;
	transform_1(2, 2) = C;
	transform_1(2, 3) = D;
	//    (row, column)
	// Print the transformation  
	std::cout << "Method #1: using a Matrix4f\n" << endl;
	std::cout << transform_1 << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
	//multiviewerndt(cloud, transformed_cloud); 


	pcl::PointXYZ min;//用于存放三个轴的最小值
	pcl::PointXYZ max;//用于存放三个轴的最大值
	pcl::getMinMax3D(*cloud, min, max);

	int t_num = transformed_cloud->points.size();
	float total = 0;
	for (int i = 0; i < t_num; ++i)
	{
		total += transformed_cloud->points[i].z;


	}
	float s_area = (max.x - min.x) * (max.y - min.y);
	float volume = total / t_num * s_area;
	cout << "Calculated Volume: (cm^3) " << volume * 1000 << endl;
	cout << "Calculated Volume: (m^3) " << volume / 1000 << endl;


	return volume;
}


double Volume::CalcualteV2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//transform_1(2, 0) =-0.135176; transform_1(2, 1) = 0.0436936; transform_1(2, 2) = 0.985858; transform_1(2, 3) = 38.0787;
	transform_1(2, 0) = A;
	transform_1(2, 1) = B;
	transform_1(2, 2) = C;
	transform_1(2, 3) = D;
	//    (row, column)
	// Print the transformation  
	std::cout << "Method #1: using a Matrix4f\n" << endl;
	std::cout << transform_1 << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
	//multiviewerndt(cloud, transformed_cloud); 


	pcl::PointXYZ min;//用于存放三个轴的最小值
	pcl::PointXYZ max;//用于存放三个轴的最大值
	pcl::getMinMax3D(*transformed_cloud, min, max);
	
	//start to mesh the pcl
	const int nsize = 100;
	int ncount[nsize][nsize] = { {0} };
	double grid[nsize][nsize] = { {0} };
	double dx = (max.x - min.x) / nsize;
	double dy = (max.y - min.y) / nsize;
	int t_num = transformed_cloud->points.size();
	double grid2[nsize][nsize] = { {0} };
	for (int i = 0; i < t_num; ++i)
	{
		int j = int((transformed_cloud->points[i].x-min.x)/dx-0.001);
		int k = int((transformed_cloud->points[i].y-min.y)/dy-0.001);
		grid[j][k] += transformed_cloud->points[i].z;
		grid2[j][k] += (A * cloud->points[i].x + B * cloud->points[i].y + C * cloud->points[i].z + D) / sqrt(A * A + B * B + C * C);
		ncount[j][k] += 1;
		
	}

	double t2=0;
	int ct=0;
	double total = 0.0;
	double total2 = 0.0;
	for (int i = 0; i < nsize; ++i) {
		
		for (int j = 0; j < nsize; ++j) {
			if (ncount[i][j] != 0 && abs(grid[i][j] / ncount[i][j])>=0.2 && abs(grid2[i][j] / ncount[i][j])>=0.2) {
				total += grid[i][j] / ncount[i][j];
				total2 += grid2[i][j] / ncount[i][j];
				t2 += grid[i][j];
				ct += ncount[i][j];
			}

		}
	}

	cout << max.z <<"  minz:  "<< min.z << endl;
	float s_area = (max.x - min.x) * (max.y - min.y);
	float volume = total * s_area / nsize / nsize;
	float volume2 = total2 * s_area / nsize / nsize;
	cout << "Calculated Volume: (cm^3) " << volume * 1000 << endl;
	cout << "Calculated Volume22: (cm^3) " << volume2 * 1000 << endl;

	
	return volume;
}

int main()
{   
	const string cloudname = "triangle2.pcd";  // add the cropped cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloudname, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file xxxxxx.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->size() << " data points from: " << cloudname << std::endl;
	//findplane(cloud);
	Volume v;
	v.A =A;
	v.B = B;
	v.C = C;
	v.D = D;
	float calculated_volume = v.CalcualteV(cloud);
	float v2 = v.CalcualteV2(cloud);

	cout <<"result using method1"<< calculated_volume/1000<< "m^3" << endl;
	cout << "result using method2 "<<v2/1000  << " m^3" << endl;
	
	return 0;	
	
}

int croppcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	// 切割点云
	float x_min = -2;
	float y_min = -2;
	float z_min = -38;
	float x_max = 10;
	float y_max = 10;
	float z_max = -30;

	pcl::CropBox<pcl::PointXYZ> clipper;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr body{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_body{ new pcl::PointCloud<pcl::PointXYZ> };//指针还是对象，有时候只能指针，有时候都行。报错就换。
	pcl::CropBox<pcl::PointXYZ> box_filter;//滤波器对象
	box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.（
	box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
	clipper.setNegative(false);//是保留立方体内的点而去除其他点，还是反之。false是将盒子内的点去除，默认为false
	box_filter.setInputCloud(cloud);//输入源
	box_filter.filter(*filtered_body);//滤它！
	cout << "started to show" << endl;
	multiviewerndt(filtered_body, filtered_body);

	cout << "finished" << endl;
	cout << "finished" << endl;
	return 0;
	
}
int findplane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
	cout << "find plan" << endl;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 1cm
	ne.setRadiusSearch(1);
	//ne.setKSearch(20);
	ne.compute(*normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	//（2）use RANSAC to get the plane
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	pcl::PCDWriter writer;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型
	seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数
	seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法
	seg.setMaxIterations(500); //设置迭代的最大次数
	//   0.5
	seg.setDistanceThreshold(0.5); //设置内点到模型的距离允许最大值
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);

	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
	A = coefficients_plane->values[0];
	B = coefficients_plane->values[1];
	C = coefficients_plane->values[2];
	D = coefficients_plane->values[3];

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	extract.filter(*cloud_plane);
	multiviewerndt(cloud, cloud_plane);
	return 1;

}

int multiviewerndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	

	cout << "Start to show cloud" << endl;
	// 初始化点云可视化对象  // multi show 
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色

	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(cloud1, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud1, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	// 对转换后的源点云着色 (green)可视化.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(cloud2, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud2, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "output cloud");
	//viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud);
	// 启动可视化
	viewer_final->addCoordinateSystem(1.0);  //显示XYZ指示轴
	viewer_final->initCameraParameters();   //初始化摄像头参数

	// 等待直到可视化窗口关闭
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}
