#include"head.h"

int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//! ------------------------------读入点云------------------------------

	//if (pcl::io::loadPCDFile("test.pcd", *cloud) < 0) {
	//	PCL_ERROR("\a点云文件不存在！\n");
	//	system("pause");
	//}

	//! ------------------------------制作点云------------------------------
	const int maxSize = 1450;
	cloud->points.clear();
	cloud->points.resize(maxSize);

	//! 填充点云 -> 噪声
	for (int i = 0; i < 150; ++i) {
		cloud->points[i].x = rand() % 100;
		cloud->points[i].y = rand() % 100;
		cloud->points[i].z = rand() % 100;
	}
	//! 填充点云 -> 圆环
	for (int i = 150; i < 350; ++i) {
		if (i % 2 == 1) {
			cloud->points[i].x = cos(1.8 * (i - 50)) * 15 + 30;
			cloud->points[i].y = sin(1.8 * (i - 50)) * 15 + 30;
			cloud->points[i].z = 25;
		}
		else {
			cloud->points[i].x = cos(1.8 * (i - 50)) * 15 + 30 + rand() % 3;
			cloud->points[i].y = sin(1.8 * (i - 50)) * 15 + 30 + rand() % 3;
			cloud->points[i].z = 25 + rand() % 3;
		}
	}
	//! 填充点云 -> 直线
	for (int i = 350; i < 450; ++i) {
		if (i % 2 == 1) {
			cloud->points[i].x = i - 350;
			cloud->points[i].y = i - 350;
			cloud->points[i].z = i - 350;
		}
		else {
			cloud->points[i].x = i - 350 + rand() % 3;
			cloud->points[i].y = i - 350 + rand() % 3;
			cloud->points[i].z = i - 350 + rand() % 3;
		}

	}
	//! 填充点云 -> 平面
	for (int i = 450; i < 600; ++i) {
		if (i % 2 == 1) {
			cloud->points[i].x = rand() % 100;
			cloud->points[i].y = rand() % 100;
			cloud->points[i].z = 35;
		}
		else {
			cloud->points[i].x = rand() % 100;
			cloud->points[i].y = rand() % 100;
			cloud->points[i].z = 35 + rand() % 10;
		}

	}
	//! 填充点云 -> 球
	for (int i = 650; i < cloud->points.size(); ++i) {
		if (i < 850) {
			if (i % 3 == 1 && i % 3 == 2) {
				cloud->points[i].x = cos(1.2 * (i - 650)) * 15 + 90;
				cloud->points[i].y = sin(1.2 * (i - 650)) * 15 + 60;
				cloud->points[i].z = 60;
			}
			else {
				cloud->points[i].x = cos(3.6 * (i - 650)) * 15 + 90 + rand() % 3;
				cloud->points[i].y = sin(3.6 * (i - 650)) * 15 + 60 + rand() % 3;
				cloud->points[i].z = 60 + rand() % 3;
			}

		}
		else if (i >= 850 && i < 1050) {
			if (i % 3 == 1 && i % 3 == 2) {
				cloud->points[i].x = cos(1.2 * (i - 850)) * 15 + 90;
				cloud->points[i].y = 60;
				cloud->points[i].z = sin(1.2 * (i - 850)) * 15 + 60;
			}
			else {
				cloud->points[i].x = cos(3.6 * (i - 850)) * 15 + 90 + rand() % 3;
				cloud->points[i].y = 60 + rand() % 3;
				cloud->points[i].z = sin(3.6 * (i - 850)) * 15 + 60 + rand() % 3;
			}

		}
		else if (i >= 1050 && i < 1250) {
			if (i % 3 == 1 && i % 3 == 2) {
				cloud->points[i].x = 90;
				cloud->points[i].y = cos(1.2 * (i - 1050)) * 15 + 60;
				cloud->points[i].z = sin(1.2 * (i - 1050)) * 15 + 60;
			}
			else {
				cloud->points[i].x = 90 + rand() % 3;
				cloud->points[i].y = cos(3.6 * (i - 1050)) * 15 + 60 + rand() % 3;
				cloud->points[i].z = sin(3.6 * (i - 1050)) * 15 + 60 + rand() % 3;
			}

		}
	}
	//! 填充点云 -> 平面补充
	for (int i = 1250; i < 1450; ++i) {
		cloud->points[i].x = rand() % 100;
		cloud->points[i].y = rand() % 100;
		cloud->points[i].z = 35;
	}
	//! ------------------------------点云结束------------------------------

	cout << "->加载数据点的个数：" << cloud->points.size() << endl;
	std::vector<double> vPara;

	//vPara = RANSAC_Plane(cloud);
	//vPara = _Plane(cloud);

	//vPara = RANSAC_Circle3D(cloud);
	//vPara = _Circle3D(cloud);

	//vPara = RANSAC_Line(cloud);
	//vPara = _Line(cloud);

	//vPara = RANSAC_Sphere(cloud);
	vPara = _Sphere(cloud);

	//vPara = RANSAC_Circle2D(cloud);
	//vPara = _Circle2D(cloud);

	//vPara = RANSAC_Cylinder(cloud);
	//vPara = _Cylinder(cloud);

	for (int i = 0; i < vPara.size(); i++) {
		cout << vPara[i] << endl;
	}

	return 0;
}