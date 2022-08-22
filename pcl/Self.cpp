#include"head.h"

std::vector<double> _Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold, double P) {
	const int maxSize = cloud->points.size();
	//! 初始化最优点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_best(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_best->points.clear();
	cloud_best->points.resize(maxSize);
	//! 设置最大迭代值 ；最终结果的可信率
	//int N = 1e4; double P = 0.99;
	//! 平面方程 => Ax + By + Cz + D = 0
	double A = 0, B = 0, C = 0, D = 0;
	//! 最优
	double best_A = 0, best_B = 0, best_C = 0, best_D = 0;
	double distance = 0;
	//! 距离阈值
	//float threshold = 0.01;
	//! 票数
	int ticket = 0, best_ticket = 0;
	//! 迭代次数
	double plane_N = N;
	//! 取样次数
	int sample_count = 0;
	//! 外点率
	double e = 0;
	//! 内点索引
	std::vector<int> inliers;
	//! 每次随机获取的值
	int point_A = 0;
	int point_B = 0;
	int point_C = 0;
	double index = 0;

	//! 开始 RANSAC
	while (plane_N > sample_count) {

		//! 三点确定一个平面
		srand(time(0) + index++);
		point_A = rand() % maxSize;
		srand(time(0) + index++);
		point_B = rand() % maxSize;
		srand(time(0) + index++);
		point_C = rand() % maxSize;

		//! 计算平面方程参数
		A = (cloud->points[point_C].y - cloud->points[point_A].y) * (cloud->points[point_C].z - cloud->points[point_A].z) - (cloud->points[point_B].z - cloud->points[point_A].z) * (cloud->points[point_C].y - cloud->points[point_A].y);
		B = (cloud->points[point_C].x - cloud->points[point_A].x) * (cloud->points[point_B].z - cloud->points[point_A].z) - (cloud->points[point_B].x - cloud->points[point_A].x) * (cloud->points[point_C].z - cloud->points[point_A].z);
		C = (cloud->points[point_B].x - cloud->points[point_A].x) * (cloud->points[point_C].y - cloud->points[point_A].y) - (cloud->points[point_C].x - cloud->points[point_A].x) * (cloud->points[point_B].y - cloud->points[point_A].y);
		D = -(cloud->points[point_A].x * A + cloud->points[point_A].y * B + cloud->points[point_A].z * C);

		//! 初始化票数 及 关于传入点云的内点索引数组
		ticket = 0;
		inliers.clear();

		//! 遍历所有点，判断该点距离 是否在 由三点确定平面的阈值内
		for (int l = 0; l < cloud->points.size(); l++) {
			distance = abs(cloud->points[l].x * A - cloud->points[l].y * B + C * cloud->points[l].z + D) / (sqrt(A * A + B * B + C * C));
			if (distance < threshold) {
				inliers.push_back(l);
				ticket++;
			}
		}

		//! 如果得票高于历史最高 则更新
		if (ticket > best_ticket) {
			best_ticket = ticket;
			best_A = A;
			best_B = B;
			best_C = C;
			best_D = D;
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_best);
			e = 1 - ((double)ticket / (double)cloud->points.size());
			double temp_N = ComputeN(e, P, 3);

			//! 防止计算后 大于 最大迭代预设值
			if (temp_N < plane_N) {
				plane_N = temp_N;
			}
		}
		//! 记录迭代次数
		sample_count++;
	}

	cout << "plane_N = " << plane_N << endl;
	cout << "sample_count = " << sample_count << endl;
	cout << "best_A = " << best_A << endl;
	cout << "best_B = " << best_B << endl;
	cout << "best_C = " << best_C << endl;
	cout << "best_D = " << best_D << endl;
	cout << "best_ticket = " << best_ticket << endl;
	view(cloud, cloud_best);

	std::vector<double> vPara;
	vPara.push_back(best_A);
	vPara.push_back(best_B);
	vPara.push_back(best_C);
	vPara.push_back(best_D);
	return vPara;
}

std::vector<double> _Line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold, double P) {
	const int maxSize = cloud->points.size();
	//! 初始化最优点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_best(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_best->points.clear();
	cloud_best->points.resize(maxSize);
	//! 设置最大迭代值 ；最终结果的可信率
	//int N = 1e4; double P = 0.99;
	//! 平面方程 => Ax + By + Cz + D = 0
	double A = 0, B = 0, C = 0, D = 0;
	//! 最优
	Eigen::Vector3d best_p1;
	Eigen::Vector3d best_p2;
	double distance = 0;
	//! 距离阈值
	//float tolerance = 0.01;
	//! 票数
	int ticket = 0, best_ticket = 0;
	//! 迭代次数
	double line_N = N;
	//! 取样次数
	int sample_count = 0;
	//! 内点率
	double e = 0;
	//! 内点索引
	std::vector<int> inliers;
	std::vector<int> best_inliers;
	//! 每次随机获取的值
	int point_A = 0;
	int point_B = 0;
	double index = 0;

	//! 开始 RANSAC
	while (line_N > sample_count) {

		//! 三点确定一个平面
		srand(time(0) + index++);
		point_A = rand() % maxSize;
		srand(time(0) + index++);
		point_B = rand() % maxSize;

		//! 直线上 点 A
		Eigen::Vector3d p1 = { cloud->points[point_A].x, cloud->points[point_A].y, cloud->points[point_A].z };
		//! 直线上 点 B
		Eigen::Vector3d p2 = { cloud->points[point_B].x, cloud->points[point_B].y, cloud->points[point_B].z };

		//! 初始化票数 及 关于传入点云的内点索引数组
		ticket = 0;
		inliers.clear();

		//! 遍历所有点，判断该点距离 是否在 由三点确定平面的阈值内
		for (int l = 0; l < cloud->points.size(); l++) {
			Eigen::Vector3d p0 = { cloud->points[l].x, cloud->points[l].y, cloud->points[l].z };

			distance = (p1 - p0).cross(p2).norm() / p2.norm();

			if (distance < threshold) {
				inliers.push_back(l);
				ticket++;
			}
		}

		//! 如果得票高于历史最高 则更新
		if (ticket > best_ticket) {
			best_ticket = ticket;
			best_p1 = p1;
			best_p2 = p2;

			best_inliers.clear();
			best_inliers = inliers;

			e = 1 - ((double)ticket / (double)cloud->points.size());
			double temp_N = ComputeN(e, P, 2);

			//! 防止计算后 大于 最大迭代预设值
			if (temp_N < line_N) {
				line_N = temp_N;
			}
		}
		//! 记录迭代次数
		sample_count++;
	}

	//cout << "line_N = " << line_N << endl;
	//cout << "sample_count = " << sample_count << endl;
	//cout << "best_p1 = " << endl << best_p1 << endl;
	//cout << "best_p2 = " << endl << best_p2 << endl;
	//cout << "best_ticket = " << best_ticket << endl;

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, best_inliers, *cloud_best);
	view(cloud, cloud_best);

	//! 返回值 -> 最优两点坐标
	std::vector<double> vPara;
	vPara.push_back(best_p1[0]);
	vPara.push_back(best_p1[1]);
	vPara.push_back(best_p1[2]);
	vPara.push_back(best_p2[0]);
	vPara.push_back(best_p2[1]);
	vPara.push_back(best_p2[2]);
	return vPara;
}

std::vector<double> _Circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold, double P) {
	//todo 1. 计算三点所在平面得出平面方程并保留该平面所有点 (参数 A B C 为垂直于该平面向量)
	//todo 2. 计算两直线交点得到圆心，半径
	//todo    2.1 计算两线段中点
	//todo    2.2 计算过该中点且与线段垂直的直线
	//todo 3. 计算距离并筛点统计

	int maxSize = cloud->points.size();

	//! 可变参数（传入） -> 最大迭代次数  置信度  距离阈值
	//double N = 10000; double P = 0.99; double tolerance = 0.1;

	//! 存贮三点确定的平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_plane->points.clear();
	cloud_plane->points.resize(maxSize);

	//! 存储最优内点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_best(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_best->points.resize(maxSize);
	cloud_best->points.clear();
	//! 随机因子
	int ranFactor = 0;
	//! 当前距离
	double distance = 0;


	//! 票数
	int ticket = 0, best_ticket = 0;
	//! 取样次数
	int sample_count = 0;
	//! 内点率
	double e = 0;
	//! 迭代次数
	double circle_N = N;

	//! 当前计算圆参数
	double center_x = 0, center_y = 0, center_z = 0, r = 0;
	//! 最优 圆点 + 半径 
	double best_center_x = 0, best_center_y = 0, best_center_z = 0, best_r = 0;

	//! 平面局内点
	std::vector<int> inliers_plane;
	//! 圆环局内点
	std::vector<int> inliers_circle;

	//! 随机三点
	int point_A = 0;
	int point_B = 0;
	int point_C = 0;

	//! 平面参数
	double plane_A = 0;
	double plane_B = 0;
	double plane_C = 0;
	double plane_D = 0;

	//! 开始迭代计算
	while (circle_N > sample_count) {
		srand(time(0) + ranFactor++);
		point_A = rand() % maxSize;

		srand(time(0) + ranFactor++);
		point_B = rand() % maxSize;

		srand(time(0) + ranFactor++);
		point_C = rand() % maxSize;

		//顶点
		Eigen::Vector3d A(cloud->points[point_A].x, cloud->points[point_A].y, cloud->points[point_A].z);
		Eigen::Vector3d B(cloud->points[point_B].x, cloud->points[point_B].y, cloud->points[point_B].z);
		Eigen::Vector3d C(cloud->points[point_C].x, cloud->points[point_C].y, cloud->points[point_C].z);

		//! 计算平面
		plane_A = (C[1] - A[1]) * (C[2] - A[2]) - (B[2] - A[2]) * (C[1] - A[1]);
		plane_B = (C[0] - A[0]) * (B[2] - A[2]) - (B[0] - A[0]) * (C[2] - A[2]);
		plane_C = (B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]);
		plane_D = -(A[0] * plane_A + A[1] * plane_B + A[2] * plane_C);

		//! 遍历平面点 计算圆环局内点
		inliers_plane.clear();
		for (int l = 0; l < cloud->points.size(); l++) {
			double distance_plane = abs(cloud->points[l].x * plane_A - cloud->points[l].y * plane_B + plane_C * cloud->points[l].z + plane_D) / (sqrt(plane_A * plane_A + plane_B * plane_B + plane_C * plane_C));
			if (distance_plane < threshold) {
				inliers_plane.push_back(l);
			}
		}
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers_plane, *cloud_plane);

		//todo 直线参数方程 line_1
		//todo x = x1 + m1t
		//todo y = y1 + n1t
		//todo z = z1 + l1t

		//! 中点
		Eigen::Vector3d center_AB = (A + B) / 2;
		Eigen::Vector3d center_AC = (A + C) / 2;

		double m1 = plane_B * (A[2] - B[2]) - plane_C * (A[1] - B[1]);
		double m2 = plane_B * (A[2] - C[2]) - plane_C * (A[1] - C[1]);
		double n1 = plane_C * (A[0] - B[0]) - plane_A * (A[2] - B[2]);
		double n2 = plane_C * (A[0] - C[0]) - plane_A * (A[2] - C[2]);
		double l1 = plane_A * (A[1] - B[1]) - plane_B * (A[0] - B[0]);
		double l2 = plane_A * (A[1] - C[1]) - plane_B * (A[0] - C[0]);
		double t = (m2 * (center_AB[1] - center_AC[1]) - n2 * (center_AB[0] - center_AC[0])) / (m1 * n2 - m2 * n1);

		//! 圆参数方程
		center_x = center_AB[0] + m1 * t;
		center_y = center_AB[1] + n1 * t;
		center_z = center_AB[2] + l1 * t;
		r = sqrt(pow((A[0] - center_x), 2) + pow((A[1] - center_y), 2) + pow((A[2] - center_z), 2));

		inliers_circle.clear();
		ticket = 0;
		for (int l = 0; l < inliers_plane.size(); l++) {
			distance = abs(sqrt(0
				+ pow(((double)cloud_plane->points[l].x - center_x), 2)
				+ pow(((double)cloud_plane->points[l].y - center_y), 2)
				+ pow(((double)cloud_plane->points[l].z - center_z), 2)) - r);

			if (distance < threshold) {
				inliers_circle.push_back(l);
				ticket++;
			}
		}

		if (ticket > best_ticket) {
			best_ticket = ticket;
			//cout << "best_ticket = " << best_ticket << endl;
			best_center_x = center_x;
			best_center_y = center_y;
			best_center_z = center_z;
			best_r = r;
			cloud_best->points.clear();
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud_plane, inliers_circle, *cloud_best);
			e = 1 - ((double)ticket / (double)maxSize);

			//! 计算动态迭代次数
			double temp_N = ComputeN(e, P, 3);

			if (temp_N < circle_N) {
				circle_N = temp_N;
			}
		}
		sample_count++;
		//cout << "circle_N = " << circle_N << endl;
		//cout << "sample_count = " << sample_count << endl;
		//cout << "-----------------" << endl;
	}

	//cloud_best->points[0].x = best_center_x;
	//cloud_best->points[0].y = best_center_y;
	//cloud_best->points[0].z = best_center_z;
	//cout << "best_ticket = " << best_ticket << endl;
	view(cloud, cloud_best);

	//! 法向量取模
	double normal_plane_A = plane_A / sqrt(pow(plane_A, 2) + pow(plane_B, 2) + pow(plane_C, 2));
	double normal_plane_B = plane_B / sqrt(pow(plane_A, 2) + pow(plane_B, 2) + pow(plane_C, 2));
	double normal_plane_C = plane_C / sqrt(pow(plane_A, 2) + pow(plane_B, 2) + pow(plane_C, 2));

	//! 返回参数 -> 圆心 + 半径 + 法向量
	std::vector<double> vPara;
	vPara.push_back(best_center_x);
	vPara.push_back(best_center_y);
	vPara.push_back(best_center_z);
	vPara.push_back(best_r);
	vPara.push_back(normal_plane_A);
	vPara.push_back(normal_plane_B);
	vPara.push_back(normal_plane_C);

	return vPara;
}

std::vector<double> _Sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double N, double threshold, double P) {
	//todo 1. 计算三点所在平面得出平面方程并保留该平面所有点 (参数 A B C 为垂直于该平面向量)
	//todo 2. 计算两直线交点得到圆心，半径
	//todo    2.1 计算两线段中点
	//todo    2.2 计算过该中点且与线段垂直的直线
	//todo 3. 计算距离并筛点统计

	int maxSize = cloud->points.size();

	//! 可变参数（传入） -> 最大迭代次数  置信度  距离阈值
	//double N = 10000; double P = 0.99; double tolerance = 0.8;

	//! 存储最优内点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_best(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_best->points.resize(maxSize);
	cloud_best->points.clear();
	//! 随机因子
	int ranFactor = 0;
	//! 当前距离
	double distance = 0;


	//! 票数
	int ticket = 0, best_ticket = 0;
	//! 取样次数
	int sample_count = 0;
	//! 内点率
	double e = 0;
	//! 迭代次数
	double circle_N = N;

	//! 当前计算圆参数
	double center_x = 0, center_y = 0, center_z = 0, r = 0;
	//! 最优 圆点 + 半径 
	double best_center_x = 0, best_center_y = 0, best_center_z = 0, best_r = 0;

	//! 平面局内点
	std::vector<int> inliers_plane;
	//! 圆环局内点
	std::vector<int> inliers_circle;

	//! 随机三点
	int point_A = 0;
	int point_B = 0;
	int point_C = 0;

	//! 平面参数
	double plane_A = 0;
	double plane_B = 0;
	double plane_C = 0;
	double plane_D = 0;

	//! 开始迭代计算
	while (circle_N > sample_count) {
		srand(time(0) + ranFactor++);
		point_A = rand() % maxSize;

		srand(time(0) + ranFactor++);
		point_B = rand() % maxSize;

		srand(time(0) + ranFactor++);
		point_C = rand() % maxSize;

		//顶点
		Eigen::Vector3d A(cloud->points[point_A].x, cloud->points[point_A].y, cloud->points[point_A].z);
		Eigen::Vector3d B(cloud->points[point_B].x, cloud->points[point_B].y, cloud->points[point_B].z);
		Eigen::Vector3d C(cloud->points[point_C].x, cloud->points[point_C].y, cloud->points[point_C].z);

		//! 计算平面
		plane_A = (C[1] - A[1]) * (C[2] - A[2]) - (B[2] - A[2]) * (C[1] - A[1]);
		plane_B = (C[0] - A[0]) * (B[2] - A[2]) - (B[0] - A[0]) * (C[2] - A[2]);
		plane_C = (B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]);
		plane_D = -(A[0] * plane_A + A[1] * plane_B + A[2] * plane_C);

		//! 中点
		Eigen::Vector3d center_AB = (A + B) / 2;
		Eigen::Vector3d center_AC = (A + C) / 2;

		double m1 = plane_B * (A[2] - B[2]) - plane_C * (A[1] - B[1]);
		double m2 = plane_B * (A[2] - C[2]) - plane_C * (A[1] - C[1]);
		double n1 = plane_C * (A[0] - B[0]) - plane_A * (A[2] - B[2]);
		double n2 = plane_C * (A[0] - C[0]) - plane_A * (A[2] - C[2]);
		double l1 = plane_A * (A[1] - B[1]) - plane_B * (A[0] - B[0]);
		double l2 = plane_A * (A[1] - C[1]) - plane_B * (A[0] - C[0]);
		double t = (m2 * (center_AB[1] - center_AC[1]) - n2 * (center_AB[0] - center_AC[0])) / (m1 * n2 - m2 * n1);

		//! 圆参数方程
		center_x = center_AB[0] + m1 * t;
		center_y = center_AB[1] + n1 * t;
		center_z = center_AB[2] + l1 * t;
		r = sqrt(pow((A[0] - center_x), 2) + pow((A[1] - center_y), 2) + pow((A[2] - center_z), 2));

		inliers_circle.clear();
		ticket = 0;
		for (int l = 0; l < cloud->points.size(); l++) {
			distance = abs(sqrt(0
				+ pow(((double)cloud->points[l].x - center_x), 2)
				+ pow(((double)cloud->points[l].y - center_y), 2)
				+ pow(((double)cloud->points[l].z - center_z), 2)) - r);

			if (distance < threshold) {
				inliers_circle.push_back(l);
				ticket++;
			}
		}

		if (ticket > best_ticket) {
			best_ticket = ticket;
			//cout << "best_ticket = " << best_ticket << endl;
			best_center_x = center_x;
			best_center_y = center_y;
			best_center_z = center_z;
			best_r = r;
			cloud_best->points.clear();
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers_circle, *cloud_best);
			e = 1 - ((double)ticket / (double)maxSize);

			//! 计算动态迭代次数
			double temp_N = ComputeN(e, P, 3);

			if (temp_N < circle_N) {
				circle_N = temp_N;
			}
		}
		sample_count++;
		//cout << "circle_N = " << circle_N << endl;
		//cout << "sample_count = " << sample_count << endl;
		//cout << "-----------------" << endl;
	}

	//cloud_best->points[0].x = best_center_x;
	//cloud_best->points[0].y = best_center_y;
	//cloud_best->points[0].z = best_center_z;
	//cout << "best_ticket = " << best_ticket << endl;

	//! 返回参数 -> 圆心 + 半径 + 法向量
	std::vector<double> vPara;
	vPara.push_back(best_center_x);
	vPara.push_back(best_center_y);
	vPara.push_back(best_center_z);
	vPara.push_back(best_r);
	view(cloud, cloud_best);

	return vPara;
}

std::vector<double> _Plane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {

	double A = (p3[1] - p1[1]) * (p3[2] - p1[2]) - (p2[2] - p1[2]) * (p3[1] - p1[1]);
	double B = (p3[0] - p1[0]) * (p2[2] - p1[2]) - (p2[0] - p1[0]) * (p3[2] - p1[2]);
	double C = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]);
	double D = -(p1[0] * A + p1[1] * B + p1[2] * C);

	std::vector<double> vPara;
	vPara.push_back(A);
	vPara.push_back(B);
	vPara.push_back(C);
	vPara.push_back(D);
	return vPara;
}

double ComputeN(double ratioE, double probability, int sampleNumber) {
	return log(1 - probability) / log(1 - (pow(1 - ratioE, sampleNumber)));
}