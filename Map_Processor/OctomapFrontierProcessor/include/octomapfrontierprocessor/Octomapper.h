#ifndef OCTOMAPPER_H
#define OCTOMAPPER_H
#include "nav_msgs/Odometry.h"
#include "octomapfrontierprocessor/Frontier_Processor.h"
#include "octomap/OcTree.h"
#include <memory>
#include <ros/ros.h>
#include <octomapfrontierprocessor/TypeDefs.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


class Octomapper {
    
    public:
        Octomapper();
        ~Octomapper();
        bool Initialize(const ros::NodeHandle& n);
        std::vector<ros::AsyncSpinner> setAsynchSpinners(ros::NodeHandle& _nh);
        void UpdateOctomap(const PointCloudF::ConstPtr& points);

    
    private:
        std::string name_;
        std::string fixed_frame_id_,base_frame_id_,lidar_frame_id_;
        // octomap_msgs::Octomap octomap_msg;
        octomap_msgs::Octomap::Ptr octomap_msg = boost::make_shared<octomap_msgs::Octomap>();
        ros::Publisher octomap_pub_;

        ros::CallbackQueue new_points_queue_,new_pose_queue_;
        int new_points_queue_size_,new_pose_queue_size_;
        void NewPointCallback(const PointCloudF::ConstPtr& msg);
        ros::Subscriber new_points_sub_,new_pose_sub_;
        
        float octomap_resolution_, prob_hit_, prob_miss_;
        std::shared_ptr<octomap::OcTree> octomap_tree;
        Boundingbox_ bbx;
        SensorParams3DLidar_t sensor_params;
        bool publish_octomap_, acquired_transform_,boundingbox_enforced_=false,enforce_bounding_box_;
        octomap::point3d BBX_MIN,BBX_MAX;
        
        std::vector<Eigen::Vector3f>* directions;
        octomap::KeyRay keyray;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        octomap::point3d lidar_point;

        bool LoadParameters(const ros::NodeHandle& _nh);
        void updateFreeSpace();
        void initialize3DLidarLUT(std::vector<Eigen::Vector3f> &directions, const SensorParams3DLidar_t sensor_params);
        void publishOctomap();
        bool boundOctomap();
        void setNewPointsScan(ros::NodeHandle& _nh);
        void setPose(ros::NodeHandle& _nh);
        bool RegisterCallbacks(const ros::NodeHandle& n);
        void NewPoseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
        Frontier_Processor frontier_processor;
};

#endif