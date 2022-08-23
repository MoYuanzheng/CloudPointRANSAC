#pragma once
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>				
#include <pcl/segmentation/sac_segmentation.h>	
#include <pcl/visualization/cloud_viewer.h>		

#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <iostream>
#include <vector>
#include <random>
#include <ctime>

using namespace std;
using Eigen::Vector3d;

//std::vector<double> RANSAC_Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
//std::vector<double> RANSAC_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.01);
//std::vector<double> RANSAC_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e5, double threshold = 0.1);
//std::vector<double> RANSAC_Circle2D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
//std::vector<double> RANSAC_Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
//std::vector<double> RANSAC_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double Rmin, double Rmax, double N = 1e4, double threshold = 0.1);

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> inliers);
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_best);
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

std::vector<double> _Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.01, double P = 0.99);
std::vector<double> _Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1, double P = 0.99);
std::vector<double> _Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.01, double P = 0.99);
std::vector<double> _Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.2, double P = 0.99);

std::vector<double> _Plane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);
double ComputeN(double ratioE, double P, int sampleNumber);



class affine_RANSAC
{
public:
	//affine_RANSAC();
	std::vector<double> RANSAC_Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
	std::vector<double> RANSAC_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.01);
	std::vector<double> RANSAC_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e5, double threshold = 0.1);
	std::vector<double> RANSAC_Circle2D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
	std::vector<double> RANSAC_Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N = 1e4, double threshold = 0.1);
	std::vector<double> RANSAC_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double Rmin, double Rmax, double N = 1e4, double threshold = 0.1);
	std::vector<int> SplitPonitCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<Eigen::Vector3d> vertex);

private:

};

