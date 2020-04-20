#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <sstream>
#include <boost/thread.hpp>
#include "pcl_head.h"
#include "object-svm.h"
#include "lib.h"
#include <fstream>

static int pcdnum=1935;
static int maxnum=0;
char filename[100];
char outname[100];
static int first=0;
pcl::PointXYZ last_center;
std::string in_path="/media/cyber-z/E/Pedestrian detection/Offline-pcd/Stage-3/Original(const instensity+depth)/4";
std::string out_path="/media/cyber-z/E/label/module4";
std::stringstream ss;

std::vector<std::vector<double>>Svm_Predict_(SVM svm_tof,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::visualization::PCLVisualizer* viewer_Step)
{
    std::vector<std::vector<double>> write_label;
    svm_tof.original_cloud = cloud;
    svm_tof.Predict_Init();
    svm_tof.Load_SVM_Model();
    svm_tof.Clear_Vector();
    svm_tof.Filter();
    svm_tof.Cluster();
    if(!svm_tof.error_flag){
    for(int i=0;i<svm_tof.cluster_number;i++)
    {
        svm_tof.Boundary(svm_tof.cluster_vector[i].makeShared());
        if(svm_tof.error_flag || svm_tof.boundary_cloud->size()<svm_tof.exsamplenum)
            svm_tof.delete_num.push_back(i);
        else  svm_tof.Sample();
    }}
    svm_tof.Error_Delete();
    svm_tof.Object_Predict(first,last_center);
    first = svm_tof.first_flag;
    last_center = svm_tof.last_center;
    write_label = svm_tof.write_label;
    for(int i=0;i<svm_tof.object_leader.size()/2;i++)
        Draw_main_3D_box_s(svm_tof.object_leader[2*i],svm_tof.object_leader[2*i+1],viewer_Step);
    for(int i=0;i<svm_tof.object_person.size()/2;i++)
        Draw_3D_box_s(5+i,svm_tof.object_person[2*i],svm_tof.object_person[2*i+1],viewer_Step);
    return write_label;
}

void write_kitti(std::string out_path,std::vector<std::vector<double>>label)
{
    ofstream f;
    f.open(out_path.c_str(),ios::out);
    std::string class_;
    for(int i=0;i<label.size();i++)
    {
        if (int(label[i][0])==0) class_="Leader";
        else if(int(label[i][0])==1) class_="Person";
        else class_="DontCare";
        f<<class_<<" ";
        f<<setiosflags(ios::fixed)<<setprecision(2);
        for(int j=1;j<label[i].size();j++)
        {
            f<<label[i][j]<<" ";
        }
        f<<std::endl;
    }
    std::cout<<out_path<<" complished...."<<std::endl;
    f.close();
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* viewer_void)
{
    SVM svm_tof;
    svm_tof.Load_SVM_Model();
    std::vector<std::vector<double>> label;
    int corrent_num=pcdnum;
    std::string cache_input;
    std::string cache_output;
    cache_input=in_path;
    cache_output=out_path;
    char start[10];
    pcl::visualization::PCLVisualizer* viewer_s =
        static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (event.keyDown()) 
    {
        viewer_s->removeAllShapes();
        if (event.getKeySym() == "Left") 
        {
            corrent_num=pcdnum-1;
            sprintf(start,"/%u.pcd",corrent_num);
            cache_input+=start;
            if (pcl::io::loadPCDFile(cache_input, *original_cloud_ptr)!=-1) 
            {
                viewer_s->updatePointCloud<pcl::PointXYZ>(original_cloud_ptr);
                label=Svm_Predict_(svm_tof,original_cloud_ptr,viewer_s);
            }
        }
        else if (event.getKeySym() == "Right") 
        {
            corrent_num=pcdnum+1;
            sprintf(start,"/%u.pcd",corrent_num);
            cache_input+=start;
            if (pcl::io::loadPCDFile(cache_input, *original_cloud_ptr)!=-1) 
            {
                label=Svm_Predict_(svm_tof,original_cloud_ptr,viewer_s);
                viewer_s->updatePointCloud<pcl::PointXYZ>(original_cloud_ptr);
            }
            if(corrent_num>maxnum){
                sprintf(start,"/%u.txt",corrent_num);
                cache_output+=start;
                write_kitti(cache_output,label);
                maxnum=corrent_num;
            }
        }
        else if (event.getKeySym() == "Down") 
        {
            sprintf(start,"/%u.pcd",corrent_num);
            cache_input+=start;
            if (pcl::io::loadPCDFile(cache_input, *original_cloud_ptr)!=-1) 
            {
                label=Svm_Predict_(svm_tof,original_cloud_ptr,viewer_s);
                viewer_s->updatePointCloud<pcl::PointXYZ>(original_cloud_ptr);
            }
            sprintf(start,"/%u.txt",corrent_num);
            cache_output+=start;
            write_kitti(cache_output,label);
        }
        else if (event.getKeySym() == "Up") 
        {
            sprintf(start,"/%u.pcd",corrent_num);
            cache_input+=start;
            if (pcl::io::loadPCDFile(cache_input, *original_cloud_ptr)!=-1) 
            {
                label=Svm_Predict_(svm_tof,original_cloud_ptr,viewer_s);
                viewer_s->updatePointCloud<pcl::PointXYZ>(original_cloud_ptr);
            }
            sprintf(start,"/%u.txt",corrent_num);
            cache_output+=start;
            std::remove(cache_output.c_str());
        }
        ss<<"corrent:"<<corrent_num<<"  max:"<<maxnum;
        if(ss.str()!="")
                viewer_s->addText(ss.str(),500, 300,"text",0);
        ss.str("");
    }
    pcdnum=corrent_num;
}

int main(int argc, char **argv)
{
    SVM svm;
    ros::init(argc,argv,"pcl_show");
    ros::NodeHandle n;
    char start[10];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer* viewer_Step(new pcl::visualization::PCLVisualizer("viewer"));
    viewer_Step->setBackgroundColor(0,0,0);
    viewer_Step->addPointCloud<pcl::PointXYZ>(cloud);
    viewer_Step->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  
    viewer_Step->setCameraPosition(0,0,-1,0,-1,0);
    std::string filename=in_path;
    viewer_Step->registerKeyboardCallback(keyboardEventOccurred,(void*)viewer_Step);
    sprintf(start,"/%u.pcd",pcdnum);
    filename+=start;
    if (pcl::io::loadPCDFile(filename, *cloud)==-1) 
    {
        std::cerr << "ERROR: Cannot open file "<<std::endl;
    }
    else
    {
        viewer_Step->updatePointCloud<pcl::PointXYZ>(cloud);
        std::vector<std::vector<double>> label;
        svm.Load_SVM_Model();
        label = Svm_Predict_(svm,cloud,viewer_Step);
        sprintf(start,"/%u.txt",pcdnum);
        write_kitti(out_path+start,label);
        maxnum = pcdnum;
        if (svm.person_flag == 1)
            first=1;
        ss<<"corrent:"<<pcdnum<<"  max:"<<maxnum;
        viewer_Step->addText(ss.str(),500, 300,"text",0);
        ss.str("");
        while (!viewer_Step->wasStopped()) 
        {
            viewer_Step->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}