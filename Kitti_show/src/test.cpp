#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <sstream>
#include <boost/thread.hpp>
#include "pcl_head.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle n;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("test"));
    viewer->setBackgroundColor(0,0,0);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->addCoordinateSystem();
    std::string filename;
    std::string filename1;
    while (!viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_kitti;
        pcl::PointCloud<pcl::PointXYZ> cloud_cache_2;
        pcl::PointCloud<pcl::PointXYZ> cloud_cache_3;
        filename="/media/cyber-z/E/test/velodyne/0.pcd";
        filename1="/media/cyber-z/E/test/output/0.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud_kitti) == -1){
            std::cerr << "ERROR: Cannot open file "<<std::endl;
            break;}
        else
        {
            pcl::io::loadPCDFile<pcl::PointXYZ>(filename1, cloud_cache_2);
            // for(int i=0;i<cloud_cache_2.size();i++){
            //     pcl::PointXYZ P;
            //     P.x = cloud_cache_2.points[i].z;
            //     P.y = -cloud_cache_2.points[i].x;
            //     P.z = -cloud_cache_2.points[i].y;
            //     cloud_cache_3.points.push_back (P);}
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singlecolor(cloud_cache_2.makeShared(),0,255,0);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_kitti.makeShared());
            viewer->addPointCloud<pcl::PointXYZ>(cloud_cache_2.makeShared(),singlecolor,"tof");
            // viewer->updatePointCloud<pcl::PointXYZ>(cloud);
            while (!viewer->wasStopped()){
            viewer->spinOnce(100);}
        }
    }
}