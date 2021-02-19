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
using namespace std;

int plctopcd(string type, string file);
int user_data;
int multiviewerndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	
	viewer.setBackgroundColor(0, 0, 0.7);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

void getFiles1(string path, vector<string>& files)
{
	//文件句柄  
	//long hFile = 0;  //win7
	intptr_t hFile = 0;   //win10
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		// "\\*"是指读取文件夹下的所有类型的文件，若想读取特定类型的文件，以png为例，则用“\\*.png”
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles1(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(path + "\\" + fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
int findplane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
float A=0.0;
float B=0.0;
float C=0.0;
float D=0.0;
int main()
{   
	const string cloudname = "vertical_tria.pcd";  // add the cropped cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloudname, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan1.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->size() << " data points from: " << cloudname << std::endl;

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




	findplane(cloud);
	// cloud transform accouding to surface
	
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//transform_1(2, 0) =-0.135176; transform_1(2, 1) = 0.0436936; transform_1(2, 2) = 0.985858; transform_1(2, 3) = 38.0787;

	transform_1(2, 0) =A;
	transform_1(2, 1) = B;
	transform_1(2, 2) = C;
	transform_1(2, 3) = D;
	//    (row, column)

	// Print the transformation  
	printf("Method #1: using a Matrix4f\n");
	std::cout << transform_1 << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
	//multiviewerndt(cloud, transformed_cloud); 
	float xmin = transformed_cloud->points[0].x;
	float ymin = transformed_cloud->points[0].y;
	float xmax = transformed_cloud->points[0].x;
	float ymax = transformed_cloud->points[0].y;
	int t_num = transformed_cloud->points.size();
	float total = 0;
	for (int i = 0; i < t_num ; ++i)
	{
		total+= transformed_cloud->points[i].z;
		if (xmin > transformed_cloud->points[i].x) {
			xmin = transformed_cloud->points[i].x;
		}
		if (xmax < transformed_cloud->points[i].x) {
			xmax = transformed_cloud->points[i].x;
		}
		if (ymin > transformed_cloud->points[i].y) {
			ymin = transformed_cloud->points[i].y;
		}
		if (ymax < transformed_cloud->points[i].y) {
			ymax = transformed_cloud->points[i].y;
		}
				
	}
	float s_area = (xmax - xmin) * (ymax - ymin);
	float volume = total / t_num * s_area;
	cout << "Calculated Volume: (cm^3) " << volume*1000 << endl;
	cout << "Calculated Volume: (m^3) " << volume/1000 << endl;
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
int findboundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) {
	//calculate boundary;
	/*
	pcl::PointCloud<pcl::Boundary> boundary;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud(cloud_plane);
	est.setInputNormals(normals_plane);
	est.setSearchMethod(tree_plane);
	est.setKSearch(50); //一般这里的数值越高，最终边界识别的精度越好
	pcl::search::KdTree<pcl::PointXYZ>));
	est.compute(boundary);*/
	return 0;
}
int multiviewerndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	

	cout << "show cloud" << endl;
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

int plc_txt_to_pck(const char* open_txt , const char* save_pcd, string type)
{
	// 读取txt文件
	int num_txt;
	FILE* fp_txt;
	
	fp_txt = fopen(open_txt, "r");
	pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	
	
	
	
	//定义一种类型表示TXT中xyz  xyz
	if (type == "xyz") 
	{

		typedef struct TXT_Point_XYZ
		{
			double x;
			double y;
			double z;
		}TOPOINT_XYZ;

		TXT_Point_XYZ txt_points;
		vector<TXT_Point_XYZ> my_vTxtPoints;
		if (fp_txt)
		{
			// push all x y zs to the back of the container in the memory 
			while (fscanf(fp_txt, "%lf %lf %lf", &txt_points.x, &txt_points.y, &txt_points.z) != EOF)
			{//将点存入容器尾部
				my_vTxtPoints.push_back(txt_points);
			}
		}
		else
			cout << "读取txt文件失败" << endl;

		num_txt = my_vTxtPoints.size();
		
		//写入点云数据
		// cloud is a pointer
		
		cloud->width = num_txt;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			cloud->points[i].x = my_vTxtPoints[i].x;
			cloud->points[i].y = my_vTxtPoints[i].y;
			cloud->points[i].z = my_vTxtPoints[i].z;
		}
	}

	pcl::io::savePCDFileASCII(save_pcd, *cloud);
	cout << "从 txt_pcd.txt读取" << cloud->points.size() << "点写入txt_pcd.pcd" << endl;

	//打印出写入的点
	cout << "_________________________________" << endl;
	for (size_t i = 0; i < 5; ++i)
		//for (size_t i = 0; i < cloud->points.size(); ++i)
		cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << endl;
	cout << "转换txt文件成功" << endl;
	return 0;
	
	
}
int plctopcd(string type, string file) {

	
	fstream modelRead;
	pcl::PCDWriter writer;
	modelRead.open(file, std::ios_base::in);
	if (type == "xyz") 
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointXYZ pclPnt;
		while (!modelRead.eof())
		{
			modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z;
			cloud.push_back(pclPnt);
			
		}
		modelRead.close();
		
		
		string::size_type iPos = file.find_last_of('\\') + 1;
		string filename = file.substr(iPos, file.length() - iPos);
		cout << filename << endl;
		string name = filename.substr(0, filename.rfind("."));
		cout << name << endl;

		writer.write("D:/pointcloud/testpcl/banocular_camera_test/"+name+".pcd", cloud);

	}
	else if (type == "xyznormal") {

		//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//带法线的点云
		pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
		pcl::PointNormal pclPnt;
		while (!modelRead.eof())
		{
			modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z >> pclPnt.normal_x >> pclPnt.normal_y >> pclPnt.normal_z;
			cloud_with_normals.push_back(pclPnt);
		}
		modelRead.close();
		string::size_type iPos = file.find_last_of('\\') + 1;
		string filename = file.substr(iPos, file.length() - iPos);
		cout << filename << endl;
		string name = filename.substr(0, filename.rfind("."));
		cout << name << endl;
		writer.write("D:/pointcloud/testpcl/ "+name+".pcd", cloud_with_normals);
		cout << "finish writing" << endl;
		
	}

	
	
	return 0;
	
}
const char* getfilename(const char p[]) 
{
	int x = strlen(p);
	char ch = '\\';
	const char* q = strrchr(p, ch) + 1;
	return q;
	/*

	char path_buffer[_MAX_PATH] = "D:\\soft\\programming\\vmware.exe";
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	_splitpath(path_buffer, drive, dir, fname, ext);

	printf("Drive:%s\n file name: %s\n file type: %s\n", drive, fname, ext);
	strcat(fname, ext);
	printf("File name with extension :%s\n", fname);
	
	*/
}