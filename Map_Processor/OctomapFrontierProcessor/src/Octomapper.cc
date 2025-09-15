#include "octomapfrontierprocessor/Octomapper.h"
#include "octomap_msgs/Octomap.h"
#include "ros/console.h"

#include "ros/node_handle.h"
#include <chrono>
#include <ros/subscribe_options.h>


Octomapper::Octomapper()
: octomap_tree(std::make_shared<octomap::OcTree>(0.4)) ,
    directions(new std::vector<Eigen::Vector3f>())
{
  acquired_transform_ = false;
}

Octomapper::~Octomapper() {
}


bool Octomapper::Initialize(const ros::NodeHandle& n)
{
  name_ = ros::names::append(n.getNamespace(), "octomap_frontier_processor");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n))
  {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool Octomapper::RegisterCallbacks(const ros::NodeHandle& n)
{
  ros::NodeHandle nl(n);
  frontier_processor.Initialize(n,octomap_tree);
  octomap_pub_ = nl.advertise<octomap_msgs::Octomap>("octomap", 10, true);
  return true;
}

std::vector<ros::AsyncSpinner> Octomapper::setAsynchSpinners(ros::NodeHandle& _nh) {
  std::vector<ros::AsyncSpinner> async_spinners;
  // Pose spinner
  {
        ros::AsyncSpinner spinner_pose(1, &this->new_pose_queue_);
        async_spinners.push_back(spinner_pose);
        setPose(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for pose");
    }
  // Lidar spinner
  {
    ros::AsyncSpinner spinner_points(1, &this->new_points_queue_);
    async_spinners.push_back(spinner_points);
    setNewPointsScan(_nh);
  }
  return async_spinners;
}

void Octomapper::setNewPointsScan(ros::NodeHandle& _nh) {
  // Create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts = ros::SubscribeOptions::create<PointCloudF>(
      "NEW_POINTS_TOPIC",                                // topic name
      new_points_queue_size_,                            // queue length
      boost::bind(&Octomapper::NewPointCallback, this, _1), // callback
      ros::VoidPtr(),     // tracked object, we don't need one thus NULL
      &this->new_points_queue_ // pointer to callback queue object
  );
  this->new_points_sub_ = _nh.subscribe(opts);
}

void Octomapper::setPose(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        "POSE_TOPIC",                                // topic name
        new_pose_queue_size_,                            // queue length
        boost::bind(&Octomapper::NewPoseCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_pose_queue_ // pointer to callback queue object
    );
    this->new_pose_sub_ = _nh.subscribe(opts);
}

void Octomapper::NewPoseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    if(odom)
    {
      frontier_processor.odom_vec.push_back(*odom);
    }
}

void Octomapper::NewPointCallback(const PointCloudF::ConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();
    UpdateOctomap(msg);//updates octomap, frontiers and clusters
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    ROS_INFO_STREAM("Time elapsed (millisecs) Octomap + Frontier detection : "<<duration.count());
}


void Octomapper::initialize3DLidarLUT(std::vector<Eigen::Vector3f> &directions, const SensorParams3DLidar_t sensor_params) 
{
    directions.clear();

    const int                                       rangeCount         = sensor_params.horizontal_rays;
    const int                                       verticalRangeCount = sensor_params.vertical_rays;
    std::vector<std::tuple<double, double, double>> coord_coeffs;
    const double                                    minAngle = 0.0;
    const double                                    maxAngle = 2.0 * M_PI;

    const double verticalMinAngle = -sensor_params.vertical_fov / sensor_params.vertical_fov_scale_down;
    const double verticalMaxAngle = sensor_params.vertical_fov / sensor_params.vertical_fov_scale_up;

    const double yDiff = maxAngle - minAngle;
    const double pDiff = verticalMaxAngle - verticalMinAngle;

    double yAngle_step = yDiff / (rangeCount - 1);

    double pAngle_step;
    if (verticalRangeCount > 1)
        pAngle_step = pDiff / (verticalRangeCount - 1);
    else
        pAngle_step = 0;

    coord_coeffs.reserve(rangeCount * verticalRangeCount);

    for (int i = 0; i < rangeCount; i++) {
        for (int j = 0; j < verticalRangeCount; j++) {

        // Get angles of ray to get xyz for point
        const double yAngle = i * yAngle_step + minAngle;
        const double pAngle = j * pAngle_step + verticalMinAngle;
        const double x_coeff = cos(pAngle) * cos(yAngle);
        const double y_coeff = cos(pAngle) * sin(yAngle);
        const double z_coeff = sin(pAngle);
        coord_coeffs.push_back({x_coeff, y_coeff, z_coeff});
        }
    }
    int it = 0;
    for (int row = 0; row < verticalRangeCount; row++) {
        for (int col = 0; col < rangeCount; col++) {
            // Access the tuple and unpack it manually
            const auto& coeffs = coord_coeffs.at(col * verticalRangeCount + row);
            const float x_coeff = std::get<0>(coeffs);
            const float y_coeff = std::get<1>(coeffs);
            const float z_coeff = std::get<2>(coeffs);
            Eigen::Vector3f direction(x_coeff, y_coeff, z_coeff);
            directions.push_back(direction);
        }
    }
}


bool Octomapper::LoadParameters(const ros::NodeHandle& n)
{
  ROS_INFO_STREAM(n.getNamespace());
  if (!n.getParam("frame_id/fixed", fixed_frame_id_))
    {
      ROS_INFO("Failed to load fixed_frame_id_");
      return false;
    }
  if (!n.getParam("frame_id/base", base_frame_id_))
    {
      ROS_INFO("Failed to load base_frame_id_");
      return false;
    }
  if (!n.getParam("frame_id/lidar", lidar_frame_id_))
    {
      ROS_INFO("Failed to load lidar_frame_id_");
      return false;
    }
  if (!n.getParam("frame_id/bbx", bbx.frame))
    {
      ROS_INFO("Failed to load frame");
      return false;
    }
  
  if (!n.getParam("octomap/publish", publish_octomap_))
    {
      ROS_INFO("Failed to load publish_octomap_");
      return false;
    }
  if (!n.getParam("octomap/enforce_bounding_box",enforce_bounding_box_))
    {
      ROS_INFO("Failed to load enforce_bounding_box_");
      return false;
    }
  if (!n.getParam("octomap/resolution", octomap_resolution_))
    {
      ROS_INFO("Failed to load octomap_resolution_");
      return false;
    }
  if (!n.getParam("octomap/prob_hit", prob_hit_))
    {
      ROS_INFO("Failed to load prob_hit_");
      return false;
    }
  if (!n.getParam("octomap/prob_miss", prob_miss_))
    {
      ROS_INFO("Failed to load prob_miss_");
      return false;
    }
  double temporary;
  if (!n.getParam("octomap/bbx_min_x", temporary))
    {
      ROS_INFO("Failed to load bbx_min_x");
      return false;
    }
  bbx.bbx_min.setX(temporary);
  if (!n.getParam("octomap/bbx_min_y", temporary))
    {
      ROS_INFO("Failed to load bbx_min_y");
      return false;
    }
  bbx.bbx_min.setY(temporary);
  if (!n.getParam("octomap/bbx_min_z", temporary))
    {
      ROS_INFO("Failed to load bbx_min_z");
      return false;
    }
  bbx.bbx_min.setZ(temporary);
  if (!n.getParam("octomap/bbx_max_x", temporary))
    {
      ROS_INFO("Failed to load bbx_max_x");
      return false;
    }
  bbx.bbx_max.setX(temporary);
  if (!n.getParam("octomap/bbx_max_y", temporary))
    {
      ROS_INFO("Failed to load bbx_max_y");
      return false;
    }
  bbx.bbx_max.setY(temporary);
  if (!n.getParam("octomap/bbx_max_z", temporary))
    {
      ROS_INFO("Failed to load bbx_max_z");
      return false;
    }
  bbx.bbx_max.setZ(temporary);
  
  octomap_tree->clear();
  octomap_tree->setResolution(octomap_resolution_);
  octomap_tree->setProbHit(prob_hit_);
  octomap_tree->setProbMiss(prob_miss_);

  if (!n.getParam("lidar/max_range", sensor_params.max_range))
    {
      ROS_INFO("Failed to load max_range");
      return false;
    }
  if (!n.getParam("lidar/free_ray_distance", sensor_params.free_ray_distance))
    {
      ROS_INFO("Failed to load free_ray_distance");
      return false;
    }
  if (!n.getParam("lidar/vertical_fov_angle", sensor_params.vertical_fov))
    {
      ROS_INFO("Failed to load vertical_fov");
      return false;
    }
  if (!n.getParam("lidar/vertical_fov_scale_down", sensor_params.vertical_fov_scale_down))
    {
      ROS_INFO("Failed to load vertical_fov_scale_down");
      return false;
    }
  if (!n.getParam("lidar/vertical_fov_scale_up", sensor_params.vertical_fov_scale_up))
    {
      ROS_INFO("Failed to load vertical_fov_scale_up");
      return false;
    }
  if (!n.getParam("lidar/horizontal_rays", sensor_params.horizontal_rays))
    {
      ROS_INFO("Failed to load horizontal_rays");
      return false;
    }
  if (!n.getParam("lidar/vertical_rays", sensor_params.vertical_rays))
    {
      ROS_INFO("Failed to load vertical_rays");
      return false;
    }
  if (!n.getParam("lidar/unknown_rays/update_free_space", sensor_params.update_free_space))
    {
      ROS_INFO("Failed to load update_free_space");
      return false;
    }
  if (!n.getParam("lidar/unknown_rays/free_ray_distance_unknown", sensor_params.free_ray_distance_unknown))
    {
      ROS_INFO("Failed to load free_ray_distance_unknown");
      return false;
    }
  initialize3DLidarLUT(*directions,sensor_params);
  return true;
}

void Octomapper::UpdateOctomap(const PointCloudF::ConstPtr& points)
{
  try {
    listener.lookupTransform(fixed_frame_id_, lidar_frame_id_, ros::Time(0), transform);
    lidar_point.x() = transform.getOrigin().x();
    lidar_point.y() = transform.getOrigin().y();
    lidar_point.z() = transform.getOrigin().z();
    // ROS_INFO("Sensor origin in 'map' frame: x=%f, y=%f, z=%f", lidar_point.x(), lidar_point.y(), lidar_point.z());
    acquired_transform_ = true;
  }
    catch (tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      
  }
  if(!acquired_transform_)
      {
        for (const auto& point : points->points)
        {
          octomap::point3d octomap_point(point.x, point.y, point.z);
          octomap_tree->updateNode(octomap_point, true);
        }
      }
  else 
      {
        if(enforce_bounding_box_)
        {
            boundOctomap();
            enforce_bounding_box_=false;
        }
        octomap::Pointcloud octomap_cloud;
        for (const auto& point : points->points)
        {
          octomap::point3d octomap_point(point.x, point.y, point.z);
          octomap::point3d octomap_normal(point.normal_x, point.normal_y, point.normal_z);
          
          octomap_tree->updateNode(octomap_point,true);
          octomap_tree->insertRay(lidar_point,octomap_point);
        }
      }
    updateFreeSpace();  
    publishOctomap();
    frontier_processor.updateFrontiers(true);
    frontier_processor.publishFrontiers(fixed_frame_id_);
}

void Octomapper::publishOctomap()
{
    octomap_tree->updateInnerOccupancy();
    octomap_msg->header.frame_id = fixed_frame_id_;
    octomap_msg->header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*octomap_tree, *octomap_msg) && publish_octomap_) {
        octomap_pub_.publish(octomap_msg);
    }
}


void Octomapper::updateFreeSpace() {
  for (const auto& direction : *directions) {
      octomap::point3d end_point;
      octomap::point3d ray_direction(direction.x(), direction.y(), direction.z());
      bool hit = octomap_tree->castRay(lidar_point, ray_direction, end_point, false, sensor_params.free_ray_distance);
      if (!hit) {
        if (octomap_tree->computeRayKeys(lidar_point, end_point, keyray)) {
          for (octomap::OcTreeKey key : keyray) {
            octomap::OcTreeNode* node = octomap_tree->search(key);
            if(node == nullptr || !octomap_tree->isNodeOccupied(node))
            {
              octomap_tree->updateNode(key, false);  // false = free
              frontier_processor.updateFrontiers(false,lidar_point,end_point);
            }
          }
        }
      }
  }
}

bool Octomapper::boundOctomap()
{
    listener.lookupTransform(fixed_frame_id_, bbx.frame, ros::Time(0), transform);
    tf::Vector3 new_bbx_min = transform*bbx.bbx_min;
    tf::Vector3 new_bbx_max = transform*bbx.bbx_max;
    ROS_INFO_STREAM("bounding box min: (" 
    << new_bbx_min.getX() << ", " 
    << new_bbx_min.getY() << ", " 
    << new_bbx_min.getZ() << ")");

    ROS_INFO_STREAM("bounding box max: (" 
        << new_bbx_max.getX() << ", " 
        << new_bbx_max.getY() << ", " 
        << new_bbx_max.getZ() << ")");
    BBX_MIN.x() = new_bbx_min.getX();
    BBX_MIN.y() = new_bbx_min.getY();
    BBX_MIN.z() = new_bbx_min.getZ();
    BBX_MAX.x() = new_bbx_max.getX();
    BBX_MAX.y() = new_bbx_max.getY();
    BBX_MAX.z() = new_bbx_max.getZ();
    octomap_tree->useBBXLimit(true);
    octomap_tree->setBBXMin(BBX_MIN);
    octomap_tree->setBBXMax(BBX_MAX);
    return true;
}