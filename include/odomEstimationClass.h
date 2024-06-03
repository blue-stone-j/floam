/*
Description:
Author     : Ji Qingshi
date       :
*/

#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

// std lib
#include <string>
#include <math.h>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h> // 直通滤波
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h> // uses point neighborhood statistics to filter outlier data.
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

// ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass
{

public:
	OdomEstimationClass();

	void init(lidar::Lidar lidar_param, double map_resolution);
	void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
												 const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in);
	void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
												 const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in);
	void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap);

	Eigen::Isometry3d odom; //??? new odom
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;

private:
	// optimization variable
	double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
	Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
	Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

	Eigen::Isometry3d last_odom; //??? old odom

	// kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;

	// points downsampling before add to map
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

	// local map
	pcl::CropBox<pcl::PointXYZI> cropBoxFilter;

	// optimization count
	int optimization_count;

	// function
	void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function);
	void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function);
	void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud);
	void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
	void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_