#include "plane_fitting_ground_segmenter.h"

GroundPlaneFittingSegmenter::GroundPlaneFittingSegmenter(
    const GroundPlaneFittingSegmenterParams& params) {
  params_ = params;

  if (params.is_visualize) {
    viewer_ =
        std::make_shared<pcl::visualization::PCLVisualizer>("GPF 3D Viewer");
  }
}

void GroundPlaneFittingSegmenter::segment(
    const PointTypeCloud& cloud_in,
    std::vector<PointTypeCloudPtr>& cloud_clusters) {
  if (cloud_in.empty()) {
    return;
  }
  points_flag_.clear();
  points_flag_.resize(cloud_in.size(), UNKNOWN);
  cloud_clusters.clear();
  cloud_clusters.resize(2);

  if (params_.is_visualize) {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
  }

  PointTypeCloud cloud_obstacle, cloud_unknown, cloud_ground,
      cloud_final_obstacle;
  preprocessPoints(cloud_in, cloud_obstacle, cloud_unknown);

  model_t plane_model = iterateFitPlane(cloud_unknown);
  segmentCloud(cloud_unknown, plane_model, cloud_ground, cloud_final_obstacle);

  cloud_final_obstacle += cloud_obstacle;
  cloud_clusters[0] = cloud_ground.makeShared();
  cloud_clusters[1] = cloud_final_obstacle.makeShared();
}

void GroundPlaneFittingSegmenter::preprocessPoints(
    const PointTypeCloud& cloud_in, PointTypeCloud& cloud_obstacle,
    PointTypeCloud& cloud_unknown) {
  for (int index = 0; index < cloud_in.size(); index++) {
    const PointType& point = cloud_in[index];
    if (point.x < params_.x_min || point.x > params_.x_max ||
        point.y < params_.y_min || point.y > params_.y_max ||
        point.z < params_.z_min || point.z > params_.z_max) {
      points_flag_[index] = NOISE;
    } else {
      if (point.y > params_.ground_y_min)
        points_flag_[index] = UNKNOWN;
      else
        points_flag_[index] = OBSTACLE;
    }
  }
  std::vector<int> indices_obstacle;
  std::vector<int> indices_unknown;
  std::vector<int> indices_noise;
  for (int index = 0; index < points_flag_.size(); index++) {
    if (points_flag_[index] == UNKNOWN) {
      indices_unknown.push_back(index);
    } else if (points_flag_[index] == OBSTACLE) {
      indices_obstacle.push_back(index);
    } else if (points_flag_[index] == NOISE) {
      indices_noise.push_back(index);
    }
  }
  cloud_obstacle.clear();
  pcl::copyPointCloud(cloud_in, indices_obstacle, cloud_obstacle);
  cloud_unknown.clear();
  pcl::copyPointCloud(cloud_in, indices_unknown, cloud_unknown);

  if (params_.is_visualize) {
    PointTypeCloud cloud_noise;
    pcl::copyPointCloud(cloud_in, indices_noise, cloud_noise);
    int color_arrary[][3] = {139, 0, 0, 255, 165, 0, 50, 205, 50};
    // show cloud_obstacle
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        color_cloud_obstacle(cloud_obstacle.makeShared(), color_arrary[0][0],
                             color_arrary[0][1], color_arrary[0][2]);
    viewer_->addPointCloud<PointType>(cloud_obstacle.makeShared(),
                                      color_cloud_obstacle, "cloud_obstacle");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "cloud_obstacle");
    // show cloud_unknown
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        color_cloud_unknown(cloud_unknown.makeShared(), color_arrary[1][0],
                            color_arrary[1][1], color_arrary[1][2]);
    viewer_->addPointCloud<PointType>(cloud_unknown.makeShared(),
                                      color_cloud_unknown, "cloud_unknown");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "cloud_unknown");
    // show cloud_noise
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        color_cloud_noise(cloud_noise.makeShared(), color_arrary[2][0],
                          color_arrary[2][1], color_arrary[2][2]);
    viewer_->addPointCloud<PointType>(cloud_noise.makeShared(),
                                      color_cloud_noise, "cloud_noise");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "cloud_noise");

    viewer_->addCube(params_.x_min, params_.x_max, params_.y_min, params_.y_max,
                     params_.z_min, params_.z_max, 1.0, 0.0, 0.0, "cube1");
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube1");
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "cube1");
  }
}

GroundPlaneFittingSegmenter::model_t GroundPlaneFittingSegmenter::estimatePlane(
    const PointTypeCloud& cloud_in) {
  model_t model;

  // Create covariance matrix.
  // 1. calculate (x,y,z) mean
  float mean_x = 0., mean_y = 0., mean_z = 0.;
  for (size_t pt = 0u; pt < cloud_in.points.size(); ++pt) {
    mean_x += cloud_in.points[pt].x;
    mean_y += cloud_in.points[pt].y;
    mean_z += cloud_in.points[pt].z;
  }
  if (cloud_in.points.size()) {
    mean_x /= cloud_in.points.size();
    mean_y /= cloud_in.points.size();
    mean_z /= cloud_in.points.size();
  }
  // 2. calculate covariance
  // cov(x,x), cov(y,y), cov(z,z)
  // cov(x,y), cov(x,z), cov(y,z)
  float cov_xx = 0., cov_yy = 0., cov_zz = 0.;
  float cov_xy = 0., cov_xz = 0., cov_yz = 0.;
  for (int i = 0; i < cloud_in.points.size(); i++) {
    cov_xx += (cloud_in.points[i].x - mean_x) * (cloud_in.points[i].x - mean_x);
    cov_xy += (cloud_in.points[i].x - mean_x) * (cloud_in.points[i].y - mean_y);
    cov_xz += (cloud_in.points[i].x - mean_x) * (cloud_in.points[i].z - mean_z);
    cov_yy += (cloud_in.points[i].y - mean_y) * (cloud_in.points[i].y - mean_y);
    cov_yz += (cloud_in.points[i].y - mean_y) * (cloud_in.points[i].z - mean_z);
    cov_zz += (cloud_in.points[i].z - mean_z) * (cloud_in.points[i].z - mean_z);
  }
  // 3. setup covariance matrix Cov
  Eigen::MatrixXf Cov(3, 3);
  Cov << cov_xx, cov_xy, cov_xz, cov_xy, cov_yy, cov_yz, cov_xz, cov_yz, cov_zz;
  Cov /= cloud_in.points.size();

  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> SVD(
      Cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  model.normal = (SVD.matrixU().col(2));
  // d is directly computed substituting x with s^ which is a good
  // representative for the points belonging to the plane
  Eigen::MatrixXf mean_seeds(3, 1);
  mean_seeds << mean_x, mean_y, mean_z;
  // according to normal^T*[x,y,z]^T = -d
  model.d = -(model.normal.transpose() * mean_seeds)(0, 0);

  // ROS_WARN_STREAM("Model: " << model.normal << " " << model.d);

  return model;
}

GroundPlaneFittingSegmenter::model_t
GroundPlaneFittingSegmenter::iterateFitPlane(const PointTypeCloud& cloud_in) {
  PointTypeCloud cloud_gnds;
  std::vector<int> gnds_indices;
  model_t model;
  pcl::copyPointCloud(cloud_in, cloud_gnds);

  for (size_t iter = 0u; iter < params_.num_iter; ++iter) {
    model = estimatePlane(cloud_gnds);
    cloud_gnds.clear();
    gnds_indices.clear();

    Eigen::MatrixXf cloud_matrix(cloud_in.points.size(), 3);
    size_t pi = 0u;
    for (auto p : cloud_in.points) {
      cloud_matrix.row(pi++) << p.x, p.y, p.z;
    }
    Eigen::VectorXf dists = cloud_matrix * model.normal;
    double th_dist = -params_.th_gnds - model.d;
    for (size_t pt = 0u; pt < dists.rows(); ++pt) {
      if (dists[pt] > th_dist) {
        gnds_indices.push_back(pt);
      }
    }
    if (iter != params_.num_iter - 1) {
      pcl::copyPointCloud(cloud_in, gnds_indices, cloud_gnds);
    }
  }

  return model;
}

void GroundPlaneFittingSegmenter::segmentCloud(const PointTypeCloud& cloud_in,
                                               const model_t& plane_model,
                                               PointTypeCloud& cloud_gnds,
                                               PointTypeCloud& cloud_ngnds) {
  pcl::PointIndices::Ptr gnds_indices(new pcl::PointIndices);
  Eigen::MatrixXf cloud_matrix(cloud_in.points.size(), 3);
  size_t pi = 0u;
  for (auto p : cloud_in.points) {
    cloud_matrix.row(pi++) << p.x, p.y, p.z;
  }
  Eigen::VectorXf dists = cloud_matrix * plane_model.normal;
  double th_dist = -params_.th_gnds - plane_model.d;
  for (size_t pt = 0u; pt < dists.rows(); ++pt) {
    if (dists[pt] > th_dist) {
      gnds_indices->indices.push_back(pt);
    }
  }
  pcl::copyPointCloud(cloud_in, *gnds_indices, cloud_gnds);
  pcl::ExtractIndices<PointType> indiceExtractor;
  indiceExtractor.setInputCloud(cloud_in.makeShared());
  indiceExtractor.setIndices(gnds_indices);
  indiceExtractor.setNegative(true);
  indiceExtractor.filter(cloud_ngnds);
}
