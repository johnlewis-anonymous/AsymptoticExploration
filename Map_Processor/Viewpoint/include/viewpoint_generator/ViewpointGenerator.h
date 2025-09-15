#pragma once
#include "ros/publisher.h"
#include <octomap/OcTree.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <viewpoint_generator/Frontier.h>
#include "viewpoint_generator/FrontierArray.h"
#include "viewpoint_generator/Viewpoint.h"
#include "viewpoint_generator/ViewpointArray.h"
#include "viewpoint_generator/TypeDefs.h"



#include "nav_msgs/Odometry.h"


class ViewpointGenerator{
    public:
        ViewpointGenerator();
        ~ViewpointGenerator();
        bool Initialize(const ros::NodeHandle& n);
        std::vector<ros::AsyncSpinner>  setAsynchSpinners(ros::NodeHandle& _nh);

    private:
        std::string name_;
        bool viewpoint_enable_=false, viewpoint_publish_;
        int viewpoint_method_;
        ros::Publisher viewpoints_pub_, best_viewpoint_pub_;

        bool LoadParameters(const ros::NodeHandle& _nh);
        void updateFrontiers( bool correction=true,octomap::point3d lidar_point=octomap::point3d(0,0,0),octomap::point3d octomap_point=octomap::point3d(0,0,0),octomap::point3d octomap_normal=octomap::point3d(0,0,0));
        void publishFrontiers(std::string fixed_frame_id_);
        bool boundFrontiers(octomap::point3d point, octomap::point3d normal);
        void generateViewpoints();
        void setViewpoints(viewpoint_generator::FrontierArray::ConstPtr cluster_array);
        void calculateVoxelCount(octomap::point3d bbx_min, octomap::point3d bbx_max);
        float updateInformationGain(viewpoint_generator::Viewpoint viewpoint);
        octomap::point3d boundCube(octomap::point3d point, bool max);

        
        ros::Subscriber new_visited_viewpoints_sub_, new_pose_sub_, new_cluster_sub_, new_octomap_sub_;
        ros::CallbackQueue new_visited_viewpoints_queue_, new_pose_queue_, new_cluster_queue_, new_octomap_queue_;
        int new_visited_viewpoints_queue_size_, new_pose_queue_size_=100, new_cluster_queue_size_=1, new_octomap_queue_size_=1;

        std::vector<bool> cluster_check;
        nav_msgs::Odometry current_odom;
        
        std::shared_ptr<octomap::OcTree> octomap_tree_;
        double octree_resolution,inf_gain_cube_side, k_gain,lambda;
        viewpoint_generator::Viewpoint temporary_viewpoint;
        viewpoint_generator::Viewpoint chosen_viewpoint;
        float max_inf_gain=-100000;
        viewpoint_generator::ViewpointArrayPtr viewpoint_array;
        int temporary_voxel_count[3]={0,0,0}; //0 free 1 occupied 2 unknown
        bool calculate_bounding_box_voxel_count=true;
        bool octree_enabled=false, odom_enabled=false;
        int bounding_box_voxel_count;
        Boundingbox_ bbx;       
        
        void setPose(ros::NodeHandle& _nh);
        void setCluster(ros::NodeHandle& _nh);
        void setOctomap(ros::NodeHandle& _nh);
        void setVisitedViewpoints(ros::NodeHandle& _nh);

        void NewPoseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
        void NewClusterCallback(const boost::shared_ptr<const viewpoint_generator::FrontierArray>& msg);
        void NewOctomapCallback(const boost::shared_ptr<const octomap_msgs::Octomap>& msg);
        void NewVisitedViewpointsCallback(const viewpoint_generator::ViewpointArray::ConstPtr& msg);

        float updateDistance(float x, float y, float z,nav_msgs::Odometry odom);
        bool checkPreviousViewpoints(viewpoint_generator::Frontier temporary_viewpoint);
        float viewpoint_bucket_distance;
        bool chosen_viewpoint_set=false;
        viewpoint_generator::ViewpointArray visited_viewpoints;

};



