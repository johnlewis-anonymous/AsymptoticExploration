#pragma once
#include "tf/LinearMath/Vector3.h"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <vector>
typedef pcl::PointXYZINormal PointF;
typedef pcl::PointCloud<PointF> PointCloudF;
typedef struct
{
  double max_range;
  double free_ray_distance;
  double vertical_fov;
  double vertical_fov_scale_down;
  double vertical_fov_scale_up;
  int    vertical_rays;
  int    horizontal_rays;
  bool   update_free_space;
  bool   clear_occupied;
  double free_ray_distance_unknown;
} SensorParams3DLidar_t;
typedef struct
  {
      std::string frame;
      tf::Vector3 bbx_min,bbx_max;
  } Boundingbox_;