#include"head.h"
/*
	【平面】
	参数方程形式 => Ax + By + Cz + D = 0
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 距离阈值，用于判定局内点，默认 threshold == 0.01
	返回值：
			vector[0] => 参数 A
			vector[1] => 参数 B
			vector[2] => 参数 C
			vector[3] => 参数 D
*/
std::vector<double> RANSAC_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold ) {
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));   //针对平面模型的对象

	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);

	//与平面距离小于0.01 的点称为局内点考虑
	ransac.setDistanceThreshold(threshold);
	//设置最大迭代次数
	ransac.setMaxIterations(N);
	ransac.computeModel();
	//存储估计所得的局内点
	ransac.getInliers(inliers);

	//获得球体参数
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < model_coefficients.size(); i++) {
		vPara.push_back(model_coefficients[i]);
	}
	view(cloud, inliers);

	return vPara;
}


/*
	【球体】
	参数方程形式 => (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 距离阈值，用于判定局内点，默认 threshold == 0.01
	返回值：
			vector[0] => 球心 x 轴坐标（参数 a）
			vector[1] => 球心 y 轴坐标（参数 b）
			vector[2] => 球心 z 轴坐标（参数 c）
			vector[3] => 球心 r 半径
*/
std::vector<double> RANSAC_Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold ) {
	//创建随机采样一致性对象
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));    //针对球模型的对象

	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);

	//与平面距离小于 threshold 的点称为局内点考虑
	ransac.setDistanceThreshold(threshold);
	//设置最大迭代次数
	ransac.setMaxIterations(N);
	ransac.computeModel();
	//存储估计所得的局内点
	ransac.getInliers(inliers);
	//获得球体参数
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < model_coefficients.size(); i++) {
		vPara.push_back(model_coefficients[i]);
	}

	view(cloud, inliers);
	return vPara;
}

/*
	【直线】
	参数方程形式 => (x - point_on_line.x) / line_direction.x = (y - point_on_line.y) / line_direction.y = (z - point_on_line.z) / line_direction.z
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 距离阈值，用于判定局内点，默认 threshold == 0.01
	返回值：
			vector[0] => point_on_line.x  : 线上一个点的 X 坐标
			vector[1] => point_on_line.y  : 线上一个点的 Y 坐标
			vector[2] => point_on_line.z  : 线上一个点的 Z 坐标
			vector[3] => line_direction.x : 直线方向的  X  坐标
			vector[4] => line_direction.y : 直线方向的  Y  坐标
			vector[5] => line_direction.z : 直线方向的  Z  坐标
*/
std::vector<double> RANSAC_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N , double threshold ) {
	//创建随机采样一致性对象
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));    //针对直线模型的对象
	//创建随机采样一致性对象
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);

	//与平面距离小于 xxx 的点称为局内点考虑
	ransac.setDistanceThreshold(threshold);
	//设置最大迭代次数
	ransac.setMaxIterations(N);
	ransac.computeModel();
	//存储估计所得的局内点
	ransac.getInliers(inliers);
	//获得球体参数
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < model_coefficients.size(); i++) {
		vPara.push_back(model_coefficients[i]);
	}
	//! --------------------------------------------------------------
	view(cloud, inliers);
	return vPara;
}

/*
	柱体
	参数方程形式 =>
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 圆柱截面半径最小值
			3. 圆柱截面半径最大值
			3. 距离阈值，用于判定局内点，默认 threshold == 0.01
	返回值：
			vector[0] => point_on_axis.x  ： 位于圆柱轴上的点的 X 坐标
			vector[1] => point_on_axis.y  ： 位于圆柱轴上的点的 Y 坐标
			vector[2] => point_on_axis.z  ： 位于圆柱轴上的点的 Z 坐标
			vector[3] => axis_direction.x ： 圆柱体轴方向的   X   坐标
			vector[4] => axis_direction.y ： 圆柱体轴方向的   Y   坐标
			vector[5] => axis_direction.z ： 圆柱体轴方向的   Z   坐标
			vector[6] =>     radius       :  圆柱体截面的半径
*/
std::vector<double> RANSAC_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double Rmin, double Rmax, double N , double threshold ) {
	//-----------------------------法线估计--------------------------------
	//cout << "->正在计算法线..." << endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	// 创建法向量估计对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);						// 设置搜索方式
	ne.setInputCloud(cloud);						// 设置输入点云
	ne.setKSearch(50);								// 设置K近邻搜索点的个数
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*cloud_normals);						// 计算法向量，并将结果保存到cloud_normals中
	//=====================================================================

	//----------------------------圆柱体分割--------------------------------
	//cout << "->正在圆柱体分割..." << endl;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;		// 创建圆柱体分割对象
	seg.setInputCloud(cloud);										// 设置输入点云：待分割点云
	seg.setOptimizeCoefficients(true);								// 设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);						// 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);								// 设置采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.1);								// 设置表面法线权重系数
	seg.setMaxIterations(N);									// 设置迭代的最大次数
	seg.setDistanceThreshold(threshold);									// 设置内点到模型距离的最大值
	seg.setRadiusLimits(Rmin, Rmax);									// 设置圆柱模型半径的范围
	seg.setInputNormals(cloud_normals);								// 设置输入法向量
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);	// 保存分割结果
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);	// 保存圆柱体模型系数
	seg.segment(*inliers_cylinder, *coefficients_cylinder);			// 执行分割，将分割结果的索引保存到inliers_cylinder中，同时存储模型系数coefficients_cylinder

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < coefficients_cylinder->values.size(); i++) {
		vPara.push_back(coefficients_cylinder->values[i]);
	}
	view(cloud, inliers_cylinder->indices);

	return vPara;
}

/*
	【2D圆】（类似圆柱侧表面，z 轴面为圆柱底面）
	参数方程形式 => (x - center.x)^2 + (y - center.y)^2 = r^2 , center.z ∈ (PointsCloudMinZ(cloud), PointsCloudMaxZ(cloud))
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 距离阈值，用于判定局内点，默认 threshold == 0.1
	返回值：
			vector[0] => center.x : 圆心 x 轴坐标
			vector[1] => center.y : 圆心 y 轴坐标
			vector[2] =>  radius  : 半径 r
*/
std::vector<double> RANSAC_Circle2D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N , double threshold ) {
	//创建随机采样一致性对象
	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));
	//创建随机采样一致性对象
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);

	//设置距离阈值
	ransac.setDistanceThreshold(threshold);
	//设置最大迭代次数
	ransac.setMaxIterations(N);
	ransac.computeModel();
	//存储估计所得的局内点
	ransac.getInliers(inliers);
	//获得球体参数
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < model_coefficients.size(); i++) {
		vPara.push_back(model_coefficients[i]);
	}
	//! ---------------------------------------------------
	view(cloud, inliers);
	return vPara;
}

/*
	【3D圆】（圆环）
	参数方程形式 => (x - center.x)^2 + (y - center.y)^2 + (z - center.z)^2  = r^2 && ⚪⊥(normal.x, normal.y, normal.z)
	输入值：
			1. 点云
			2. RanSAC最大迭代次数，默认 N == 1e4
			3. 距离阈值，用于判定局内点，默认 threshold == 0.1
	返回值：
			vector[0] => center.x ： 圆心坐标 x
			vector[1] => center.y ： 圆心坐标 y
			vector[2] => center.z ： 圆心坐标 z
			vector[3] => radius   ： 半径 r
			vector[4] => normal.x ： 法线方向的 x 坐标
			vector[5] => normal.y ： 法线方向的 y 坐标
			vector[6] => normal.z ： 法线方向的 z 坐标
*/
std::vector<double> RANSAC_Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N , double threshold ) {
	//创建随机采样一致性对象
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));
	//创建随机采样一致性对象
	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);

	//设置距离阈值
	ransac.setDistanceThreshold(threshold);
	//设置最大迭代次数
	ransac.setMaxIterations(N);
	ransac.computeModel();
	//存储估计所得的局内点
	ransac.getInliers(inliers);
	//获得球体参数
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//返回参数
	std::vector<double>vPara;

	for (int i = 0; i < model_coefficients.size(); i++) {
		vPara.push_back(model_coefficients[i]);
	}
	view(cloud, inliers);
	return vPara;
}