/*
Description:
Author     : Ji Qingshi
date       :
*/

#ifndef _LASER_MAPPING_H_
#define _LASER_MAPPING_H_

// PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

// eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

// c++ lib
#include <string>
#include <math.h>
#include <vector>

#define LASER_CELL_WIDTH 50.0 // size of map grid
#define LASER_CELL_HEIGHT 50.0
#define LASER_CELL_DEPTH 50.0

// separate map as many sub point clouds

#define LASER_CELL_RANGE_HORIZONTAL 2
#define LASER_CELL_RANGE_VERTICAL 2

class LaserMappingClass
{
 public:
  LaserMappingClass( );
  void init(double map_resolution); // initialize???
  // 等距变换(Isometry Transform)可以看作是维持任意两点距离不变的仿射变换
  void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                const Eigen::Isometry3d &pose_current);
  pcl::PointCloud<pcl::PointXYZI>::Ptr getMap(void); //???

 private:
  int origin_in_map_x, origin_in_map_y, origin_in_map_z; // index of origin grid of map
  int map_width, map_height, map_depth;
  std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map; // 3d map
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

  void addWidthCellNegative(void); //???
  void addWidthCellPositive(void);
  void addHeightCellNegative(void);
  void addHeightCellPositive(void);
  void addDepthCellNegative(void);
  void addDepthCellPositive(void);
  void checkPoints(int &x, int &y, int &z);
};

#endif // _LASER_MAPPING_H_
