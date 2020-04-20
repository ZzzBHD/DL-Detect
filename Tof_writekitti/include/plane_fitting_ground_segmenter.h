#pragma once

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointTypeCloud;
typedef PointTypeCloud::Ptr PointTypeCloudPtr;

typedef struct {
  double sensor_height = 2.0;
  double y_min = -2.0;
  double y_max = 2.0;
  double ground_y_min = 1.2;
  double x_min = -10.0;
  double x_max = 10.0;
  double z_min = 2.0;
  double z_max = 13.0;
  int num_iter = 2;
  double th_gnds = 0.1;
  bool is_visualize = false;
} GroundPlaneFittingSegmenterParams;

class GroundPlaneFittingSegmenter {
 public:
  enum point_flag { UNKNOWN, NOISE, GROUND, OBSTACLE };

  typedef struct {
    Eigen::MatrixXf normal;
    double d = 0.;
  } model_t;

  inline GroundPlaneFittingSegmenter(){};

  GroundPlaneFittingSegmenter(const GroundPlaneFittingSegmenterParams& params);

  void segment(const PointTypeCloud& cloud_in,
               std::vector<PointTypeCloudPtr>& cloud_clusters);

 private:
  void preprocessPoints(const PointTypeCloud& cloud_in,
                        PointTypeCloud& cloud_obstacle,
                        PointTypeCloud& cloud_unknown);

  model_t estimatePlane(const PointTypeCloud& cloud_in);

  model_t iterateFitPlane(const PointTypeCloud& cloud_in);

  void segmentCloud(const PointTypeCloud& cloud_in, const model_t& plane_model,
                    PointTypeCloud& cloud_gnds, PointTypeCloud& cloud_ngnds);

 private:
  GroundPlaneFittingSegmenterParams params_;
  std::vector<point_flag> points_flag_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};
