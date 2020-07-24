#include <boost/program_options.hpp>
#include <iostream>     
#include <string>      
#include "pcl_head.h"  
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>

using namespace std;

static bool show_flag = false;
static int object_point_num = 25;
vector<float> box_data;

std::vector<int> Colour_Lib(int n)
{
    std::vector<int> colour;
	switch (n)
	{
	case 1:colour={255,0,0};return colour;break;//car
	case 2:colour={135,206,250};return colour;break;//van
	case 3:colour={0,255,255};return colour;break;//trunk
	case 4:colour={0,0,255};return colour;break;//Pedestrian
	case 5:colour={0,255,127};return colour;break;//Person_sitting
	case 6:colour={0,255,0};return colour;break;//Cyclist	
	case 7:colour={255,246,143};return colour;break;//Tram 	
	case 8:colour={192,255,62};return colour;break;//Misc	
	case 9:colour={255,160,122};return colour;break;//other
	case 10:colour={218,112,214};return colour;break;//temp
	case 11:colour={255,255,255};return colour;break;//temp
	default:return colour={255,255,255};return colour;
	}
}

void kittiCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    box_data.clear();
    box_data = msg->data;
    show_flag = true;
    // cout<<"Get Data!"<<box_data.size()<<endl;
}

void Draw_3Dbox(float object_box[8][3],pcl::visualization::PCLVisualizer::Ptr viewer,vector<int> colour, int obj_num)
{
   pcl::PointXYZ point1,point2;
   char label[10];
   /*
      7-----4
    6------5|
    |       |
    |       |
    | 3-----0
    2-----1/
   */
    //top
    point1.x=object_box[0][0];point1.y=object_box[0][1];point1.z=object_box[0][2];
    point2.x=object_box[1][0];point2.y=object_box[1][1];point2.z=object_box[1][2];
    sprintf(label,"%u-bottom1",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point1.x=object_box[2][0];point1.y=object_box[2][1];point1.z=object_box[2][2];
    sprintf(label,"%u-bottom2",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point2.x=object_box[3][0];point2.y=object_box[3][1];point2.z=object_box[3][2];
    sprintf(label,"%u-bottom3",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point1.x=object_box[0][0];point1.y=object_box[0][1];point1.z=object_box[0][2];
    sprintf(label,"%u-bottom4",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    //left
    point1.x=object_box[2][0];point1.y=object_box[2][1];point1.z=object_box[2][2];
    point2.x=object_box[6][0];point2.y=object_box[6][1];point2.z=object_box[6][2];
    sprintf(label,"%u-left1",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point1.x=object_box[7][0];point1.y=object_box[7][1];point1.z=object_box[7][2];
    sprintf(label,"%u-left2",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point2.x=object_box[3][0];point2.y=object_box[3][1];point2.z=object_box[3][2];
    sprintf(label,"%u-left3",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    //right
    point1.x=object_box[1][0];point1.y=object_box[1][1];point1.z=object_box[1][2];
    point2.x=object_box[5][0];point2.y=object_box[5][1];point2.z=object_box[5][2];
    sprintf(label,"%u-right1",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point1.x=object_box[4][0];point1.y=object_box[4][1];point1.z=object_box[4][2];
    sprintf(label,"%u-right2",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point2.x=object_box[0][0];point2.y=object_box[0][1];point2.z=object_box[0][2];
    sprintf(label,"%u-right3",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    //top
    point1.x=object_box[6][0];point1.y=object_box[6][1];point1.z=object_box[6][2];
    point2.x=object_box[5][0];point2.y=object_box[5][1];point2.z=object_box[5][2];
    sprintf(label,"%u-top1",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);
    point1.x=object_box[4][0];point1.y=object_box[4][1];point1.z=object_box[4][2];
    point2.x=object_box[7][0];point2.y=object_box[7][1];point2.z=object_box[7][2];
    sprintf(label,"%u-top2",obj_num);viewer->addLine(point1,point2,colour[0],colour[1],colour[2],label);

}

std::string num2type(int num)
{
    std::string type;
	switch (num)
	{
	case 1:type="Car";break;
	case 2:type="Van";break;
	case 3:type="Truck";break;
	case 4:type="Pedestrian";break;
	case 5:type="Person_sitting";break;
	case 6:type="Cyclist";break;
	case 7:type="Tram";break;
    case 8:type="Misc";break;
	default:type="Other";
	}
    return type;
}

int main (int argc, char** argv)
{
    ros::init(argc,argv,"box_show");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    string input_dir, input_type, show_type,show_num,input_filename,type_text,cloud_type;
    int show_count;
    vector<int> colour;
    ros::Subscriber sub = n.subscribe("kitti_box", 1000, kittiCallback);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("test"));
    pn.param<int>("show_count", show_count, 1);
    pn.param<std::string>("show_num_", show_num, "1");
    std::cout<<show_num<<std::endl;
    pn.param<std::string>("show_type", show_type, "single");
    pn.param<std::string>("input_type_option", input_type, "pcd");
    pn.param<std::string>("cloud_type", cloud_type, "XYZ");

    if(input_type=="pcd"){
        pn.param<std::string>("pcd_dir",input_dir, "pcd");
        input_filename = input_dir + std::string("/") + show_num + std::string(".pcd");
        cout<<input_filename<<endl;
        if (cloud_type=="XYZ"){
        if (pcl::io::loadPCDFile(input_filename, *cloud_xyz)==-1){
            std::cerr << "ERROR: Cannot open file "<<std::endl;
            exit(1);}}
        else if(cloud_type=="XYZI"){
            if (pcl::io::loadPCDFile(input_filename, *cloud_xyzi)==-1){
            std::cerr << "ERROR: Cannot open file "<<std::endl;
            exit(1);}}
    }
    else if(input_type=="bin"){
        pn.param<std::string>("bin_dir",input_dir, "bin");
        input_filename = input_dir + std::string("/") + show_num + std::string(".bin");
        fstream input(input_filename.c_str(), ios::in | ios::binary);
        if(!input.good()){
		    cerr << "Could not read file: " << input_filename << endl;
		    exit(EXIT_FAILURE);}
        input.seekg(0, ios::beg);
        int i;
	    for (i=0; input.good() && !input.eof(); i++) {
		    pcl::PointXYZ point_xyz;pcl::PointXYZI point_xyzi;
            if(cloud_type=="XYZ"){
		        input.read((char *) &point_xyz.x, 3*sizeof(float));
                cloud_xyz->push_back(point_xyz);}
            else if(cloud_type=="XYZI"){
                input.read((char *) &point_xyzi.x, 3*sizeof(float));
		        input.read((char *) &point_xyzi.intensity, sizeof(float));
                cloud_xyzi->push_back(point_xyzi);}    
	    }
	    input.close();
    }
    else
        {cout<<"Input ERROR!"<<endl; exit(1);}

    ros::spinOnce();
    viewer->setBackgroundColor(0,0,0);
    if(cloud_type=="XYZ")
        viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz);
    else if(cloud_type=="XYZI")
        viewer->addPointCloud<pcl::PointXYZI>(cloud_xyzi);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  
    // viewer->setCameraPosition(0,0,0,0,0,0);
    viewer->addCoordinateSystem(10);

    while(!viewer->wasStopped()){
    ros::spinOnce();
    if(show_flag)
    {
        int object_num = box_data.size()/object_point_num;
        float object_box[object_num][8][3];
        int object_type[object_num];
        for(int i=0; i<object_num; i++)
        {
            object_type[i] = box_data[object_point_num*i];
            for(int j=0; j<8;j++)
            {
                object_box[i][j][0]=box_data[object_point_num*i+3*j+1];
                object_box[i][j][1]=box_data[object_point_num*i+3*j+2];
                object_box[i][j][2]=box_data[object_point_num*i+3*j+3];
            }
        }
        // ROS_INFO("Data Receive!");
        viewer->removeAllShapes();
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0;i<object_num; i++){
        //     // 提取包围框中的点云(一个)
        //     pcl::PointXYZ P;
        //     for(int i=1;i<=cloud_xyzi->size();i++)
        //     {
        //         P.x = cloud_xyzi->points[i].x;
        //         P.y = cloud_xyzi->points[i].y;
        //         P.z = cloud_xyzi->points[i].z;
        //         if(P.x>=7.33193643 && P.x<=7.91036154 
        //         && P.y>=0.98975728 && P.y<=2.08022689
        //         && P.z>=-1.5 && P.z<=0.07810372){
        //         cloud_single->points.push_back (P);
        //         }
        //     }
        //     cloud_single->height=1;
        //     cloud_single->width=cloud_single->points.size();
            // pcl::io::savePCDFile("/media/cyber-z/E/test/ttt/single.pcd",*cloud_single);
            colour=Colour_Lib(int(object_type[i]));
            type_text=num2type(int(object_type[i]));
            Draw_3Dbox(object_box[i],viewer,colour,i+1);
            
        }
        viewer->spinOnce(100);
        // show_flag = false;
    }
    // else
    //   ROS_INFO("No Data!");
    }
    
}