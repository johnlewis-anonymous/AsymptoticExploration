#pragma once
#include "nav_msgs/Odometry.h"
#include <octomap/OcTree.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <unordered_set>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomapper_frontier_processor/Frontier.h>
#include <octomapper_frontier_processor/FrontierArray.h>
#include <vector>

namespace std {
    template <>
    struct hash<octomapper_frontier_processor::Frontier> {
        std::size_t operator()(const octomapper_frontier_processor::Frontier& p) const noexcept {
            //(0 1 2) (x y z)  , (3) freespace or behind obstacle
            std::size_t hx = std::hash<float>{}(p.x);
            std::size_t hy = std::hash<float>{}(p.y);
            std::size_t hz = std::hash<float>{}(p.z);
            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };
}

inline bool operator==(const octomapper_frontier_processor::Frontier& a, const octomapper_frontier_processor::Frontier& b) {
    return a.x == b.x && a.y == b.y && a.z== b.z;
}

class Frontier_Processor{
    public:
        Frontier_Processor();
        ~Frontier_Processor();
        bool Initialize(const ros::NodeHandle& n, std::shared_ptr<octomap::OcTree> tree);
        bool LoadParameters(const ros::NodeHandle& _nh);
        void updateFrontiers( bool correction=true,octomap::point3d lidar_point=octomap::point3d(0,0,0),octomap::point3d octomap_point=octomap::point3d(0,0,0),octomap::point3d octomap_normal=octomap::point3d(0,0,0));
        void publishFrontiers(std::string fixed_frame_id_);
        bool boundFrontiers(octomap::point3d point, octomap::point3d normal, float height);
        bool frontierwithinRange(const octomapper_frontier_processor::Frontier vec);
        std::shared_ptr<std::unordered_set<octomapper_frontier_processor::Frontier>> frontier_set_;
        std::vector<nav_msgs::Odometry> odom_vec;

    private:
        std::string name_;
        bool publish_frontiers_, enable_clusters;
        float z_normal_threshold_max_, z_normal_threshold_min_;
        float x_normal_threshold_max_, x_normal_threshold_min_;
        float y_normal_threshold_max_, y_normal_threshold_min_;
        float distance_within_reach;
        octomap::point3d temporary_octomap_point3d;
        
        octomap::point3d ray_direction, behind_point;
        ros::Publisher frontier_pub_;
        octomap::point3d BBX_MIN,BBX_MAX;
        std::shared_ptr<octomap::OcTree> octomap_tree;
        octomapper_frontier_processor::Frontier tempfrontier_;
        octomapper_frontier_processor::FrontierArrayPtr tempfrontierarray_;

};



