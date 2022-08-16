//#include"head.h"
//
//std::vector<double> RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int flag) {
//
//	if (flag == 1) {//! 直线
//		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
//		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
//	}
//	else if (flag == 2) {//! 平面
//		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
//	}
//	else if (flag == 3) {//! 2D圆
//		pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));
//	}
//	else if (flag == 4) {//! 3D圆
//		pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));
//	}
//	else if (flag == 5) {//! 球
//		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
//	}
//	else if (flag == 6) {//! 柱体
//		pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::PointXYZ>::Ptr
//			model(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::PointXYZ>(cloud));
//	}
//
//	//创建随机采样一致性对象
//	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
//
//	//与平面距离小于 xxx 的点称为局内点考虑
//	ransac.setDistanceThreshold(5);
//	ransac.computeModel();
//	//存储估计所得的局内点
//	ransac.getInliers(inliers);
//	//获得球体参数
//	Eigen::VectorXf model_coefficients;
//	ransac.getModelCoefficients(model_coefficients);
//
//	//返回参数
//	std::vector<double>vPara;
//
//	for (int i = 0; i < model_coefficients.size(); i++) {
//		vPara.push_back(model_coefficients[i]);
//	}
//
//
//	return vPara;
//}