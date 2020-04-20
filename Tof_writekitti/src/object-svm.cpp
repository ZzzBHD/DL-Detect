#include "object-svm.h"

SVM::SVM()
    :error_flag(0),
    person_flag(0),
    delete_flag(0),
    start_flag(0),
    first_flag(0),
    lost_flag(0),
    distance_diff(0),
    DIFF(10),
    m_y_line{1,2,2,2,2,2,2,2,2},
    dis_inf(1000),
    svm_result(0),
    cluster_number(0),
    boundary_number(0),
    search_time(0),
    lost_time(0),
    c_Tolerance(0.12),
    c_min_size(100),
    c_max_size(4000),
    b_RadiusSearch(0.3),
    ground_segmenter(params)
{
    double camera_cx,camera_cy,camera_fx,camera_fy;
    camera_cx = 487.6446798734261;
    camera_cy = 279.5953266225303;
    camera_fx = 535.9643029929152;
    camera_fy = 537.1964560834192;
    Set_Camera_Params(camera_cx,camera_cy,camera_fx,camera_fy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);original_cloud=cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_(new pcl::PointCloud<pcl::PointXYZ>);filter_cloud=filter_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr offland_cloud_(new pcl::PointCloud<pcl::PointXYZ>);offland_cloud=offland_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud_(new pcl::PointCloud<pcl::PointXYZ>);boundary_cloud=boundary_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud_(new pcl::PointCloud<pcl::PointXYZ>);sample_cloud=sample_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);m_tree=tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);m_normals=normals;
}

void SVM::Clear_Vector()
{
    cluster_vector.clear();
    sample_vector.clear();
    object_limit.clear();
    cluster_indices.clear();
    left.clear();
    right.clear();
    delete_num.clear();
    write_label.clear();
    object_leader.clear();
    object_person.clear();
}

void SVM::Filter()
{
    filter_cloud->clear();
    m_passthrough.setInputCloud(original_cloud);
    m_passthrough.setFilterFieldName("z");
    m_passthrough.setFilterLimits(4,12);//设置直通滤波器操作范围(5-12)
    m_passthrough.setFilterLimitsNegative(false);//false表示保留范围内，true表示保留范围外
    m_passthrough.filter(*filter_cloud);
    pipline(filter_cloud);
}

void SVM::Boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boundary_cloud->clear();
    boundary_number=0;
    pcl::PointCloud<pcl::Boundary> boundaries;
    m_normEst.setInputCloud(cloud);
    m_normEst.setSearchMethod(m_tree);
    m_normEst.setRadiusSearch(0);//法向估计的半径
    m_normEst.setKSearch(5);  //法向估计的点数
    m_normEst.compute(*m_normals);
    m_normEst.setViewPoint(0,0,0);
    m_est.setInputCloud(cloud);
    m_est.setInputNormals(m_normals);/*M_PI_2 */
    m_est.setSearchMethod(m_tree);
    m_est.setRadiusSearch(b_RadiusSearch);  //搜索半径，越大点越少（0.15——10ms）
    m_est.compute(boundaries);
    for (int i = 0; i<cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);
        int a = static_cast<int>(x); //该函数的功能是强制类型转换
        if (a == 1)
        {
            boundary_cloud->push_back(cloud->points[i]);
            boundary_number++;
        }
    }
    if(boundary_number==0)
    {
        std::cerr<<"No BOUNDARY!!!!!"<<std::endl;
        error_flag=1;
    }
}

void SVM::Cluster()
{
    m_tree->setInputCloud (offland_cloud);
    m_ec.setClusterTolerance (c_Tolerance); // 设置近邻搜索半径为n cm
    m_ec.setMinClusterSize (c_min_size);  //设置一个聚类最少点数目
    m_ec.setMaxClusterSize (c_max_size);//设置一个聚类最多点数目(1500)
    m_ec.setSearchMethod (m_tree);//设置点云的搜索机制
    m_ec.setInputCloud (offland_cloud);
    m_ec.extract (cluster_indices);//从点云中提取聚类
    cluster_number=cluster_indices.size();
    if(cluster_number==0)
    {
        std::cerr<<"NO OBJECT!!!!!"<<std::endl;
        error_flag=1;
    }
    else
    {
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud_cache_cluster;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cache_cluster.points.push_back (offland_cloud->points[*pit]); //*
            cloud_cache_cluster.width = cloud_cache_cluster.points.size ();
            cloud_cache_cluster.height = 1;
            cloud_cache_cluster.is_dense = true;
            pcl::getMinMax3D(cloud_cache_cluster,min,max);
            object_limit.push_back(min);//保存当前object的6个极点
            object_limit.push_back(max);
            cluster_vector.push_back(cloud_cache_cluster);
        }
    }
}

void SVM::Sample()
{
    object_cache.clear();
    m_all_point.clear();
    int gear=m_y_line.size();//层数
    for(int i=0;i<boundary_cloud->size();i++)
        m_all_point.push_back(boundary_cloud->points[i]);
    std::sort(m_all_point.begin(),m_all_point.end(),comp);
    int step=m_all_point.size()/gear;
    int k=0;
    for(int i=0;i<gear;i++)//每层分割线
    {
        for(int j=0;j<m_y_line[i];j++)//每层分割线有多少点需要采样
            object_cache.push_back (m_all_point[k+j]);
        k+=step;
    }
    sample_vector.push_back(object_cache);
}

void SVM::Set_Camera_Params(double cx,double cy,double fx,double fy)
{
    m_camera_cx=cx;
    m_camera_cy=cy;
    m_camera_fx=fx;
    m_camera_fy=fy;
}

void SVM::Load_SVM_Model()
{
    char charname1[100];char charname2[100];
    sprintf(charname1,"/media/cyber-z/E/Pedestrian detection/Offline-pcd/Stage-2/SVM-Sample/SVM-3-R.1.model");
    sprintf(charname2,"/media/cyber-z/E/Pedestrian detection/Offline-pcd/Stage-2/SVM-Sample/SVM-3-R.2.model");
    model_1=svm_load_model(charname1);
    model_2=svm_load_model(charname2);
}

void SVM::Sample_write(std::vector<pcl::PointXYZ> object_sample)
{
    for (int i=0;i<object_sample.size();i++)
    {
        m_Point[i][0]=i+1;
        m_Point[i][1]=object_sample[i].x;
        m_Point[i][2]=object_sample[i].y;
        m_Point[i][3]=object_sample[i].z;
    }
}

void SVM::Sample_SVM(double Point[][4])
{
    svmnode[exsamplenum*dim].index=-1;
    left.clear();
    right.clear();
    double head=Point[0][1];
    double middle,lev_x_diff,lev_min,lev_max;
    double lev_middle=head;//上一层的中值点，初始为头点
    int lev=2;bool abnormal_flag=0;
    for(int i=2;i<exsamplenum;i=i+2)
    {
        middle=(Point[i-1][1]+Point[i][1])/double(2);
        lev_x_diff=fabs(Point[i-1][1]-Point[i][1]);
        if(lev_x_diff<0.18)//同侧 
        {
            if(lev==2)//第二层
            {
                if(Point[i-1][1]<Point[i][1])//i点在左侧
                {
                    left.push_back(i-1);right.push_back(i);
                    if(middle<head) Point[i][1]=Point[i-1][1]+2*fabs(Point[i-1][1]-head);
                    else Point[i-1][1]=Point[i][1]-2*fabs(Point[i][1]-head);
                }
                else//i点在右侧
                {
                    left.push_back(i);right.push_back(i-1);
                    if(middle<head) Point[i-1][1]=Point[i][1]+2*fabs(Point[i][1]-head);
                    else Point[i][1]=Point[i-1][1]-2*fabs(Point[i-1][1]-head);
                }
            }
            else//非第二层
            {
                if(middle<lev_middle)//左侧
                {
                    if(Point[i-1][1]<Point[i][1])//i点在左侧
                    {
                        if((lev_x_diff>0.08)&&(Point[i][1]<lev_min))
                        {
                            left.push_back(i);right.push_back(i-1);
                            Point[i-1][1]=lev_max-fabs(Point[i][1]-lev_min);
                        }
                        else
                        {
                            left.push_back(i-1);right.push_back(i);
                            Point[i][1]=Point[i-1][1]+2*fabs(Point[i-1][1]-lev_middle);
                        }
                    }
                    else//i点在右侧
                    {
                        if((lev_x_diff>0.08)&&(Point[i-1][1]<lev_min))
                        {
                            left.push_back(i-1);right.push_back(i);
                            Point[i][1]=lev_max-fabs(Point[i-1][1]-lev_min);
                        }
                        else
                        {
                            left.push_back(i);right.push_back(i-1);
                            Point[i-1][1]=Point[i][1]+2*fabs(Point[i][1]-lev_middle);
                        }
                    }
                }
                else//右侧
                {
                    if(Point[i-1][1]<Point[i][1])//i点在左侧
                    {
                        if((lev_x_diff>0.08)&&(Point[i-1][1]>lev_max))
                        {
                            left.push_back(i);right.push_back(i-1);
                            Point[i][1]=lev_min+fabs(Point[i-1][1]-lev_max);
                        }
                        else
                        {
                            left.push_back(i-1);right.push_back(i);
                            Point[i-1][1]=Point[i][1]-2*fabs(Point[i][1]-lev_middle);
                        }
                    }
                    else//i点在右侧
                    {
                        if((lev_x_diff>0.08)&&(Point[i][1]>lev_max))
                        {
                            left.push_back(i-1);right.push_back(i);
                            Point[i-1][1]=lev_min+fabs(Point[i][1]-lev_max);
                        }
                        else
                        {
                            left.push_back(i);right.push_back(i-1);
                            Point[i][1]=Point[i-1][1]-2*fabs(Point[i-1][1]-lev_middle);
                        }
                    }
                }
            }
        }
        else//异侧
        {
            if(lev_x_diff>0.5)//异常点
            {
                if(Point[i-1][1]<Point[i][1])//i点在左侧
                {
                    left.push_back(i-1);right.push_back(i);
                    if(middle<lev_middle) Point[i-1][1]=Point[i][1]-2*fabs(Point[i][1]-lev_middle);
                    else Point[i][1]=Point[i-1][1]+2*fabs(Point[i-1][1]-lev_middle);
                }
                else//i点在右侧
                {
                    left.push_back(i);right.push_back(i-1);
                    if(middle<lev_middle) Point[i][1]=Point[i-1][1]-2*fabs(Point[i-1][1]-lev_middle);
                    else Point[i-1][1]=Point[i][1]+2*fabs(Point[i][1]-lev_middle);
                }
                abnormal_flag==1;
            }
            else//异侧非异常点
            {
                if(Point[i-1][1]<Point[i][1]) {left.push_back(i-1);right.push_back(i);}
                else {left.push_back(i);right.push_back(i-1);}
            }            
        }
        if(!abnormal_flag)
        {
            lev_min=std::min(Point[i-1][1],Point[i][1]);
            lev_max=std::max(Point[i-1][1],Point[i][1]);
            lev_middle=(Point[i-1][1]+Point[i][1])/double(2);
            abnormal_flag=0;
        }
        lev++;
    }
    // std::cout<<std::endl;
    // std::cout<<"1 "<<Point[0][1]<<" "<<Point[0][2]<<" "<<Point[0][3]<<std::endl;
    // for(int i=0;i<left.size();i++)
    //     std::cout<<left[i]+1<<" "<<Point[left[i]][1]<<" "<<Point[left[i]][2]<<" "<<Point[left[i]][3]<<std::endl;
    // for(int i=0;i<right.size();i++)
    //     std::cout<<right[i]+1<<" "<<Point[right[i]][1]<<" "<<Point[right[i]][2]<<" "<<Point[right[i]][3]<<std::endl;
    int svm_num=0;
    left.insert(left.begin(),0);
    for(int j=0;j<exsamplenum;j++)
    {
        if(svm_num<left.size()*dim)
        {
            for(int k=1;k<=dim;k++)
            {
                svmnode[svm_num].index=svm_num+1;
                svmnode[svm_num].value=Point[left[j]][k];
                svm_num++;
            }
        }
        else
        {
            for(int k=1;k<=dim;k++)
            {
                svmnode[svm_num].index=svm_num+1;
                svmnode[svm_num].value=Point[right[exsamplenum-j-1]][k];
                svm_num++;
            }
        }
    }
    // for(int i=0;i<SAMPLENUM*dim;i++)
    //     std::cout<<svmnode[i].index<<" "<<svmnode[i].value<<std::endl;
}

void SVM::Object_Predict(int first,pcl::PointXYZ center)
{
    for(int i=0;i<sample_vector.size();i++)
    {
        Sample_write(sample_vector[i]);
        Sample_SVM(m_Point);
        svm_result=svm_predict(model_1,svmnode);
        if(int(svm_result)!=1)// 2or3
        {
        svm_result=svm_predict(model_2,svmnode);
        draw_min=object_limit[2*i];
        draw_max=object_limit[2*i+1];
        if(int(svm_result)==1)//2
        {
            std::cout<<"Get Leader!"<<std::endl;
            CenterPoint(draw_min,draw_max);
            if(first==0){last_center=cache_center;}
            else last_center=center;
            // std::cout<<first<<std::endl;
            distance_diff=Position_diff(cache_center,last_center);
            if(distance_diff<dis_inf && distance_diff<DIFF)
            {
                person_flag=1;
                first_flag=1;
                l_center=cache_center;
                dis_inf=distance_diff;
                min=draw_min;
                max=draw_max;
            }
            else//被svm误分类为leader的person
            {
                std::cout<<"Person not Leader"<<std::endl;
                object_person.push_back(draw_min);
                object_person.push_back(draw_max);
                double label[16] = {1.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,
                fabs(draw_max.y-draw_min.y),
                fabs(draw_max.x-draw_min.x),
                fabs(draw_max.z-draw_min.z),
                cache_center.z,-cache_center.x,-cache_center.y,0,1};
                std::vector<double>cache_label(label,label+sizeof(label)/sizeof(label[0]));
                write_label.push_back(cache_label);
            }
        }
        else//3
        {
            // std::cout<<"Get Person!"<<std::endl;
            CenterPoint(draw_min,draw_max);
            object_person.push_back(draw_min);
            object_person.push_back(draw_max);
            double label[16] = {1.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,
            fabs(draw_max.y-draw_min.y),
            fabs(draw_max.x-draw_min.x),
            fabs(draw_max.z-draw_min.z),
            cache_center.z,-cache_center.x,-cache_center.y,0,1};
            std::vector<double>cache_label(label,label+sizeof(label)/sizeof(label[0]));
            write_label.push_back(cache_label);
        }
        }
    }

    if(person_flag==1) {last_center=l_center;lost_time=0;
    object_leader.push_back(min);
    object_leader.push_back(max);
    double label[16] = {0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,
    fabs(max.y-min.y),fabs(max.x-min.x),fabs(max.z-min.z),
    l_center.z,-l_center.x,-l_center.y,0,1};
    std::vector<double>cache_label(label,label+sizeof(label)/sizeof(label[0]));
    write_label.push_back(cache_label);
    person_flag=0;
    }
    else {lost_time++;}
    if(lost_time==LOST/2) last_center.x=0;
}

double SVM::Position_diff(pcl::PointXYZ P1,pcl::PointXYZ P2)
{
    double distance;
    distance=sqrt((P1.x-P2.x)*(P1.x-P2.x)/2.0+(P1.y-P2.y)*(P1.y-P2.y)+(P1.z-P2.z)*(P1.z-P2.z));
    return distance;
}

void SVM::CenterPoint(pcl::PointXYZ Pmax,pcl::PointXYZ Pmin)
{
    cache_center.x=((Pmax.x-Pmin.x)/2.0)+Pmin.x;
    cache_center.y=((Pmax.y-Pmin.y)/2.0)+Pmin.y;
    cache_center.z=((Pmax.z-Pmin.z)/2.0)+Pmin.z;
}

void SVM::Predict_Init()
{
    dis_inf=1000;
    z_min=1000;
    person_flag=0;
}

void SVM::Error_Delete()
{
if(delete_num.size()!=0)
{
    for(int i=0;i<delete_num.size();i++)
    {
        object_limit.erase(object_limit.begin()+2*(delete_num[i]-i));
        object_limit.erase(object_limit.begin()+2*(delete_num[i]-i));
    }
}
}

void SVM::pipline(PointTypeCloudPtr original_cloud_ptr) 
{
    offland_cloud->clear();
    // remove NAN
    PointTypeCloudPtr preprocessed_cloud_ptr(new PointTypeCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*original_cloud_ptr, *preprocessed_cloud_ptr,
                                indices);
    std::vector<PointTypeCloudPtr> cloud_cluster;
    ground_segmenter.segment(*preprocessed_cloud_ptr, cloud_cluster);

    if (cloud_cluster.size() > 0) 
        offland_cloud = cloud_cluster.at(1);
}