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
	// ��������
	cloud.width = 10000;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		////����
		float r = 100.0;
		cloud.points[i].x = 2 * r * rand() / (RAND_MAX + 1.0f) - r;
		cloud.points[i].y = sqrt(r * r - cloud.points[i].x * cloud.points[i].x) * rand() / (RAND_MAX + 1.0f) * ((2 * rand() / (RAND_MAX + 1.0f) - 1) > 0 ? 1 : -1);
		cloud.points[i].z = sqrt(r * r - cloud.points[i].x * cloud.points[i].x - cloud.points[i].y * cloud.points[i].y) * rand() / (RAND_MAX + 1.0f) * ((2 * rand() / (RAND_MAX + 1.0f) - 1) > 0 ? 1 : -1);


		//������
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
	sor.setLeafSize(5, 5, 5);// ע�ⵥλ��Ĭ��m, ��ص����¿�����mm�� ע������size��С
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

	// ���ش����ӽǵõ��ĵڶ���ɨ�����������ΪԴ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\pointcloud\\testpcl\\room_scan2.pcd", *input_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan2.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
	//���ϵĴ������������PCD�ļ��õ�����ָ�룬������׼����ɶ�Դ���Ƶ�Ŀ����ƵĲο�����ϵ�ı任����Ĺ��ƣ��õ��ڶ�����Ʊ任����һ���������ϵ�µı任����
	// �������ɨ��������ݹ��˵�ԭʼ�ߴ��10%�����ƥ����ٶȣ�ֻ��Դ���ƽ����˲�����������������
	//��Ŀ����Ʋ���Ҫ�˲�����
	//��Ϊ��NDT��normal distributions transform���㷨����Ŀ����ƶ�Ӧ�������������ݽṹ��ͳ�Ƽ��㲻ʹ�õ����㣬����ʹ�ð�����ÿ�����ص�Ԫ���еĵ��ͳ������
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// �²���filter
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size()
		<< " data points from room_scan2.pcd" << std::endl;

	// ��ʼ����̬�ֲ�(NDT)����
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	// �����������ݵĳ߶�����NDT��ز���

	ndt.setTransformationEpsilon(0.01);   //Ϊ��ֹ����������Сת������

	ndt.setStepSize(0.1);    //Ϊmore-thuente������������󲽳�

	ndt.setResolution(1.0);   //����NDT��������ṹ�ķֱ��ʣ�voxelgridcovariance��
	//���ϲ�����ʹ�÷���ߴ����������ȽϺã����������Ҫ��������һ�����ȱ��ӵ�ɨ��֮���С�����壬��Ҫ�Բ������кܴ�̶ȵ���С

	//����ƥ�������������������������Ƴ������е�������������һ����˵�������ֵ֮ǰ�Ż��������epsilon�任��ֵ����ֹ
	//������������������ܹ����ӳ����³������ֹ�����ڴ���ķ���������ʱ�����
	ndt.setMaximumIterations(35);

	ndt.setInputSource(filtered_cloud);  //���˹���Դ����
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(target_cloud);  //Ŀ�����

	// ����ʹ�û����˲�෨�õ��Ĵ��Գ�ʼ�任������
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// filtered cloud aligned with initguess ==> get output cloud
	ndt.align(*output_cloud, init_guess); // appliy the guessed matrix to the ndt
	//����ط���output_cloud������Ϊ���յ�Դ���Ʊ任����Ϊ����Ե��ƽ������˲�����
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
		<< " score: " << ndt.getFitnessScore() << std::endl;

	// ʹ�ô����ı任��Ϊ���˵�������ƽ��б任
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

	// ����ת�����Դ������Ϊ���յı任���
	pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

	// ��ʼ�����ƿ��ӻ�����  // multi show 
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);  //���ñ�����ɫΪ��ɫ

	// ��Ŀ�������ɫ���ӻ� (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	// ��ת�����Դ������ɫ (green)���ӻ�.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "output cloud");
	//viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud);
	// �������ӻ�
	viewer_final->addCoordinateSystem(1.0);  //��ʾXYZָʾ��
	viewer_final->initCameraParameters();   //��ʼ������ͷ����

	// �ȴ�ֱ�����ӻ����ڹر�
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}

int plc_txt_to_pck(const char* open_txt, const char* save_pcd)
{
	//����һ�����ͱ�ʾTXT��xyz
	typedef struct TXT_Point_XYZ
	{
		double x;
		double y;
		double z;
	}TOPOINT_XYZ;


	//��ȡtxt�ļ�
	int num_txt;
	FILE* fp_txt;
	TXT_Point_XYZ txt_points;
	vector<TXT_Point_XYZ> my_vTxtPoints;
	fp_txt = fopen(open_txt, "r");

	if (fp_txt)
	{
		// push all x y zs to the back of the container in the memory 
		while (fscanf(fp_txt, "%lf %lf %lf", &txt_points.x, &txt_points.y, &txt_points.z) != EOF)
		{//�����������β��
			my_vTxtPoints.push_back(txt_points);
		}
	}
	else
		cout << "��ȡtxt�ļ�ʧ��" << endl;

	num_txt = my_vTxtPoints.size();

	//д���������
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
	cout << "�� txt_pcd.txt��ȡ" << cloud->points.size() << "��д��txt_pcd.pcd" << endl;

	//��ӡ��д��ĵ�
	cout << "_________________________________" << endl;
	for (size_t i = 0; i < 5; ++i)
		//for (size_t i = 0; i < cloud->points.size(); ++i)
		cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << endl;
	cout << "ת��txt�ļ��ɹ�" << endl;
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