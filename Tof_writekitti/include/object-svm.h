#ifndef OBJECTSVM_ZB
#define OBJECTSVM_ZB

#include <ros/ros.h>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv_head.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/Pose2D.h>
#include "image_transport/image_transport.h"
#include "plane_fitting_ground_segmenter.h"
#include "svm.h"
#include "pcl_keypoint.h"
#include "pcl_head.h"
#include "pcl_filter.h"
#include "pcl_cluster.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <math.h>

class SVM
{
public:
    SVM();
    ~SVM(){}
    GroundPlaneFittingSegmenterParams params;
    GroundPlaneFittingSegmenter ground_segmenter;
    geometry_msgs::Pose2D m_leader;

    std::vector<int> m_y_line;
    static const int exsamplenum=17;
    static const int dim=3;
    static const int LOST=20;

    bool error_flag;
    bool person_flag;
    bool delete_flag;
    bool start_flag;
    bool lost_flag;
    bool first_flag;

    double m_camera_cx;
    double m_camera_cy;
    double m_camera_fx;
    double m_camera_fy;
    double m_Point[exsamplenum][4];
    double z_min;
    int cluster_number;
    int boundary_number;
    int lost_time;
    int svm_result;
    pcl::PointXYZ cache_center;
    pcl::PointXYZ l_center;
    pcl::PointXYZ last_center;
    pcl::PointXYZ start_center;
    pcl::PointXYZ init_center;

    std::vector<std::vector<double>> write_label;
    std::vector<pcl::PointXYZ> object_leader;
    std::vector<pcl::PointXYZ> object_person;
    pcl::PointXYZ draw_max;
    pcl::PointXYZ draw_min;
    pcl::PointXYZ min;
    pcl::PointXYZ max;

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr offland_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cluster_vector;
    std::vector<std::vector<pcl::PointXYZ>> sample_vector;
    std::vector<pcl::PointXYZ> object_limit;
    std::vector<pcl::PointXYZ> object_cache;
    std::vector<int> delete_num;
    void Set_Camera_Params(double cx,double cy,double fx,double fy);
    void Load_SVM_Model();
    void Clear_Vector();
    void Filter();
    void Cluster();
    void Boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void Error_Delete();
    void Sample();
    void Sample_write(std::vector<pcl::PointXYZ> object_sample);
    void Sample_SVM(double Point[][4]);
    void Object_Predict(int first,pcl::PointXYZ center);
    void Predict_Init();

private:
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> m_ec;
    pcl::PassThrough<pcl::PointXYZ> m_passthrough;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> m_est;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> m_normEst;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_tree;
    pcl::PointCloud<pcl::Normal>::Ptr m_normals;

    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<int> left;
    std::vector<int> right;
    std::vector<pcl::PointXYZ> m_all_point;

    int c_min_size;
    int c_max_size;
    int search_time;
    double distance_diff;
    double dis_inf;
    double DIFF;

    double c_Tolerance;
    double b_RadiusSearch;

    svm_model *model_1;
    svm_model *model_2;
    svm_node svmnode[exsamplenum*dim+1];

    double Position_diff(pcl::PointXYZ P1,pcl::PointXYZ P2);
    void CenterPoint(pcl::PointXYZ Pmax,pcl::PointXYZ Pmin);
    void pipline(PointTypeCloudPtr original_cloud_ptr); 
    static bool comp(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {return a.y < b.y;}


};

#endif