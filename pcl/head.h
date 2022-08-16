#pragma once
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/features/normal_3d.h>				// 法向量估计
#include <pcl/segmentation/sac_segmentation.h>	// 模型分割
#include <pcl/visualization/cloud_viewer.h>		// 可视化
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
//std::vector<double> RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int flag);

using namespace std;
using Eigen::Vector3d;

std::vector<double> RANSAC_Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> RANSAC_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> RANSAC_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> RANSAC_Circle2D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> RANSAC_Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> RANSAC_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> inliers);
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_best);
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


std::vector<double> _Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> _Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> _Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
std::vector<double> _Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);