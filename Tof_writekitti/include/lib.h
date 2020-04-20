#ifndef LIB
#define LIB

#include "pcl_head.h"
#include "pcl_filter.h"
#include "pcl_cluster.h"
#include <vector>

#define DIFF 1.5//最大跳变距离
#define INF 1000
#define Y_MAX 1.4//基准线

#define X_Center 0.1//初始中心点
#define Y_Center 1.5
#define Z_Center 5.7

bool comp(const pcl::PointXYZ &a, const pcl::PointXYZ &b);
double Position_diff(pcl::PointXYZ P1,pcl::PointXYZ P2);
pcl::PointXYZ CenterPoint(pcl::PointXYZ Pmax,pcl::PointXYZ Pmin);
pcl::PointCloud<pcl::PointXYZ>::Ptr FillEmptyCloud();
void Draw_2D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer);
void Draw_main_2D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer);
void Draw_3D_box(int num,pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer);
void Draw_main_3D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer);
void Draw_3D_box_s(int num,pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer* viewer);
void Draw_main_3D_box_s(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer* viewer);
void Sample_write(double Point[][4],std::vector<pcl::PointXYZ> object_sample);
#endif
