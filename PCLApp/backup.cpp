/*#include<iostream>

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
int plc_txt_to_pck(const char* open_txt = "D:\\pointcloud\\txttopcd\\txttopcd\\2019.6.1.01.txt", const char* save_pcd = "D:\\pointcloud\\testpcl\\new_pcd.pcd");
const char* getfilename(const char p[] = "D:\\SoftWare\\Adobe\\Photoshop5.exe");
int user_data;
int singlepclview();
int multiviewerndt();
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
int main()
{


	return 0;




}


int singlepclview() {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// 创建点云
	cloud.width = 10000;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		////球体
		float r = 100.0;
		cloud.points[i].x = 2 * r * rand() / (RAND_MAX + 1.0f) - r;
		cloud.points[i].y = sqrt(r * r - cloud.points[i].x * cloud.points[i].x) * rand() / (RAND_MAX + 1.0f) * ((2 * rand() / (RAND_MAX + 1.0f) - 1) > 0 ? 1 : -1);
		cloud.points[i].z = sqrt(r * r - cloud.points[i].x * cloud.points[i].x - cloud.points[i].y * cloud.points[i].y) * rand() / (RAND_MAX + 1.0f) * ((2 * rand() / (RAND_MAX + 1.0f) - 1) > 0 ? 1 : -1);


		//正方体
		//cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		//cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		//cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	pcl::io::savePCDFileASCII("D:\\pointcloud\\testpcl\\ball_white.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

	// get a pcl pointer cloud1 2 3, rgba or not rgba
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	// load the pcd to the pointer
	pcl::io::loadPCDFile("D:\\pointcloud\\testpcl\\ball_white.pcd", *cloud3);

	// get a  pcl pointer 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// creat a voxelgrid
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor2;
	// origal pcl
	sor.setInputCloud(cloud3);
	sor2.setInputCloud(cloud3);
	//filter
	sor.setLeafSize(5, 5, 5);// 注意单位，默认m, 相关点云下可能是mm， 注意设置size大小
	sor2.setLeafSize(0.5, 0.5, 0.5);
	// new pcl
	sor.filter(*filtered_cloud);
	sor2.filter(*cloud1);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");


	//blocks until the cloud is actually rendered
	//viewer.setBackgroundColor(0, 0, 0.7);

	viewer.showCloud(filtered_cloud);
	//viewer.showCloud(cloud3);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}

	system("pause");
	return(0);
}
int multiviewerndt() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\pointcloud\\testpcl\\room_scan1.pcd", *target_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan1.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << target_cloud->size() << " data points from room_scan2.pcd" << std::endl;

	// 加载从新视角得到的第二次扫描点云数据作为源点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\pointcloud\\testpcl\\room_scan2.pcd", *input_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan2.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
	//以上的代码加载了两个PCD文件得到共享指针，后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计，得到第二组点云变换到第一组点云坐标系下的变换矩阵
	// 将输入的扫描点云数据过滤到原始尺寸的10%以提高匹配的速度，只对源点云进行滤波，减少其数据量，
	//而目标点云不需要滤波处理
	//因为在NDT（normal distributions transform）算法中在目标点云对应的体素网格数据结构的统计计算不使用单个点，而是使用包含在每个体素单元格中的点的统计数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 下采样filter
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size()
		<< " data points from room_scan2.pcd" << std::endl;

	// 初始化正态分布(NDT)对象
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	// 根据输入数据的尺度设置NDT相关参数

	ndt.setTransformationEpsilon(0.01);   //为终止条件设置最小转换差异

	ndt.setStepSize(0.1);    //为more-thuente线搜索设置最大步长

	ndt.setResolution(1.0);   //设置NDT网格网格结构的分辨率（voxelgridcovariance）
	//以上参数在使用房间尺寸比例下运算比较好，但是如果需要处理例如一个咖啡杯子的扫描之类更小的物体，需要对参数进行很大程度的缩小

	//设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
	//添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
	ndt.setMaximumIterations(35);

	ndt.setInputSource(filtered_cloud);  //过滤过的源点云
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(target_cloud);  //目标点云

	// 设置使用机器人测距法得到的粗略初始变换矩阵结果
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// filtered cloud aligned with initguess ==> get output cloud
	ndt.align(*output_cloud, init_guess); // appliy the guessed matrix to the ndt
	//这个地方的output_cloud不能作为最终的源点云变换，因为上面对点云进行了滤波处理
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
		<< " score: " << ndt.getFitnessScore() << std::endl;

	// 使用创建的变换对为过滤的输入点云进行变换
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

	// 保存转换后的源点云作为最终的变换输出
	pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

	// 初始化点云可视化对象  // multi show 
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色

	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	// 对转换后的源点云着色 (green)可视化.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
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

int plc_txt_to_pck(const char* open_txt, const char* save_pcd)
{
	//定义一种类型表示TXT中xyz
	typedef struct TXT_Point_XYZ
	{
		double x;
		double y;
		double z;
	}TOPOINT_XYZ;


	//读取txt文件
	int num_txt;
	FILE* fp_txt;
	TXT_Point_XYZ txt_points;
	vector<TXT_Point_XYZ> my_vTxtPoints;
	fp_txt = fopen(open_txt, "r");

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
	pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
//}