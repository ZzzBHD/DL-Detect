#include "lib.h"

bool comp(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{     
    return a.y < b.y;
}

double Position_diff(pcl::PointXYZ P1,pcl::PointXYZ P2)
{
    double distance;
    distance=sqrt((P1.x-P2.x)*(P1.x-P2.x)/2.0+(P1.y-P2.y)*(P1.y-P2.y)+(P1.z-P2.z)*(P1.z-P2.z));
    return distance;
}

pcl::PointXYZ CenterPoint(pcl::PointXYZ Pmax,pcl::PointXYZ Pmin)
{
    pcl::PointXYZ P;
    if(Pmax.y!=0&&Pmax.y<Y_MAX)Pmax.y=Y_MAX;
    P.x=((Pmax.x-Pmin.x)/2.0)+Pmin.x;
    P.y=((Pmax.y-Pmin.y)/2.0)+Pmin.y;
    P.z=((Pmax.z-Pmin.z)/2.0)+Pmin.z;
    return P;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FillEmptyCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    empty_cloud->width    = 1;
    empty_cloud->height   = 1;
    empty_cloud->is_dense = true;
    empty_cloud->points.resize (empty_cloud->width * empty_cloud->height);
    empty_cloud->points[0].x=0;
    empty_cloud->points[0].y=0;
    empty_cloud->points[0].z=0;
    return empty_cloud;
}

void Draw_2D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer)
{
   pcl::PointXYZ point1,point2;
   if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
   //bottom
   point1.x=min.x;point1.y=min.y;point1.z=min.z;
   point2.x=max.x;point2.y=min.y;point2.z=min.z;
   viewer->addLine(point1,point2,0,0,255,"bottom");
   //left
   point1.x=min.x;point1.y=min.y;point1.z=min.z;
   point2.x=min.x;point2.y=max.y;point2.z=min.z;
   viewer->addLine(point1,point2,0,0,255,"left");
   //right
   point1.x=max.x;point1.y=max.y;point1.z=min.z;
   point2.x=max.x;point2.y=min.y;point2.z=min.z;
   viewer->addLine(point1,point2,0,0,255,"right");
   //top
   point1.x=max.x;point1.y=max.y;point1.z=min.z;
   point2.x=min.x;point2.y=max.y;point2.z=min.z;
   viewer->addLine(point1,point2,0,0,255,"top");
}

void Draw_main_2D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer)
{
   pcl::PointXYZ point1,point2;
   if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
   //bottom
   point1.x=min.x;point1.y=min.y;point1.z=min.z;
   point2.x=max.x;point2.y=min.y;point2.z=min.z;
   viewer->addLine(point1,point2,255,0,0,"m_bottom");
   //left
   point1.x=min.x;point1.y=min.y;point1.z=min.z;
   point2.x=min.x;point2.y=max.y;point2.z=min.z;
   viewer->addLine(point1,point2,255,0,0,"m_left");
   //right
   point1.x=max.x;point1.y=max.y;point1.z=min.z;
   point2.x=max.x;point2.y=min.y;point2.z=min.z;
   viewer->addLine(point1,point2,255,0,0,"m_right");
   //top
   point1.x=max.x;point1.y=max.y;point1.z=min.z;
   point2.x=min.x;point2.y=max.y;point2.z=min.z;
   viewer->addLine(point1,point2,255,0,0,"m_top");
}

void Draw_3D_box(int num,pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer)
{
 /*   
       ____5____      MAX
      /|       /|                  Y
    10 7      9 8                  |  /Z
    -----1----  |                  | /
    |  |     |  |                  |/
    3  /---6-4--/                  --------X   
    | 11     | 12
    /----2----/       MIN    
*/
    pcl::PointXYZ point1,point2;
    char label[10];int i=0;
    if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=min.z; 
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
}

void Draw_3D_box_s(int num,pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer* viewer)
{
 /*   
       ____5____      MAX
      /|       /|                  Y
    10 7      9 8                  |  /Z
    -----1----  |                  | /
    |  |     |  |                  |/
    3  /---6-4--/                  --------X   
    | 11     | 12
    /----2----/       MIN    
*/
    pcl::PointXYZ point1,point2;
    char label[10];int i=0;
    // if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=min.z; 
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,0,0,255,label);i++;
}

void Draw_main_3D_box(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer::Ptr viewer)
{
 /*   
       ____5____      MAX
      /|       /|                  Y
    10 7      9 8                  |  /Z
    -----1----  |                  | /
    |  |     |  |                  |/
    3  /---6-4--/                  --------X   
    | 11     | 12
    /----2----/       MIN    
*/
    pcl::PointXYZ point1,point2;
    char label[10];int i=0;int num=-1;
    if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=min.z; 
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
}

void Draw_main_3D_box_s(pcl::PointXYZ max,pcl::PointXYZ min,pcl::visualization::PCLVisualizer* viewer)
{
 /*   
       ____5____      MAX
      /|       /|                  Y
    10 7      9 8                  |  /Z
    -----1----  |                  | /
    |  |     |  |                  |/
    3  /---6-4--/                  --------X   
    | 11     | 12
    /----2----/       MIN    
*/
    pcl::PointXYZ point1,point2;
    char label[10];int i=0;int num=-1;
    // if(max.y!=0&&max.y<Y_MAX)max.y=Y_MAX;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=min.z; 
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=min.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=max.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=max.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=max.y;point1.z=min.z;point2.x=max.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=max.y;point1.z=min.z;point2.x=min.x;point2.y=max.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=min.x;point1.y=min.y;point1.z=min.z;point2.x=min.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
    point1.x=max.x;point1.y=min.y;point1.z=min.z;point2.x=max.x;point2.y=min.y;point2.z=max.z;
    sprintf(label,"%u-%u",num,i);viewer->addLine(point1,point2,255,0,0,label);i++;
}

void Show_single_pcd(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("test"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  
    viewer->setCameraPosition(0,0,-5,0,-1,0);
    while (!viewer->wasStopped())viewer->spinOnce(100);
}

void Show_muti_pcd(std::string filename)
{
    int count=1;
    std::string inputname;
    char num[10]; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("test"));
    viewer->setBackgroundColor(0,0,0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  
    viewer->setCameraPosition(0,0,-5,0,-1,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud);
    while(1)
    {
        sprintf(num,"/%u.pcd",count); 
        inputname=filename+num;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputname, *cloud) == -1)
            break; 
        else
        {
            viewer->updatePointCloud<pcl::PointXYZ>(cloud);
            viewer->spinOnce(100);
            count++; 
        }
    }
}

void Sample_write(double Point[][4],std::vector<pcl::PointXYZ> object_sample)//特征取三维
{
    for (int i=0;i<object_sample.size();i++)
    {
        Point[i][0]=i+1;
        Point[i][1]=object_sample[i].x;
        Point[i][2]=object_sample[i].y;
        Point[i][3]=object_sample[i].z;
    }
}