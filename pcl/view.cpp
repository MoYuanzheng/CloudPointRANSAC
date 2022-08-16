#include"head.h"

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> inliers) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_best(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_best);

	//声明视窗
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置视窗背景色
	viewer->setBackgroundColor(0, 0, 0);
	//预处理点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta(cloud, 255, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta_best(cloud_best, 0, 255, 0);
	//把点云加载到视窗
	viewer->addPointCloud(cloud, magenta, "cloud");
	viewer->addPointCloud(cloud_best, magenta_best, "cloud_best");
	//设置点云大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_best");
	//显示
	viewer->spin();
}

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_best) {
	//声明视窗
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置视窗背景色
	viewer->setBackgroundColor(0, 0, 0);
	//预处理点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta(cloud, 255, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta_best(cloud_best, 0, 255, 0);
	//把点云加载到视窗
	viewer->addPointCloud(cloud, magenta, "cloud");
	viewer->addPointCloud(cloud_best, magenta_best, "cloud_best");
	//设置点云大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_best");
	//显示
	viewer->spin();
}

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	//声明视窗
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置视窗背景色
	viewer->setBackgroundColor(0, 0, 0);
	//预处理点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta(cloud, 255, 0, 255);
	//把点云加载到视窗
	viewer->addPointCloud(cloud, magenta, "cloud");
	//设置点云大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//显示
	viewer->spin();
}