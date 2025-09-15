#include "octomapfrontierprocessor/Frontier_Processor.h"
#include "nav_msgs/Odometry.h"
#include "octomap/octomap_types.h"
#include "ros/console.h"

Frontier_Processor::Frontier_Processor()
  : frontier_set_(std::make_shared<std::unordered_set<octomapper_frontier_processor::Frontier>>())
{
    tempfrontierarray_ = boost::make_shared<octomapper_frontier_processor::FrontierArray>();
}

Frontier_Processor::~Frontier_Processor() {
}

bool Frontier_Processor::LoadParameters(const ros::NodeHandle& n)
{
    if (!n.getParam("frontier/publish", publish_frontiers_))
    {
        ROS_INFO("Failed to load publish_frontiers_");
        return false;
    }
    if (!n.getParam("frontier/distance_within_reach", distance_within_reach))
    {
        ROS_INFO("Failed to load distance_within_reach");
        return false;
    }
    if (!n.getParam("frontier/z_normal_max", z_normal_threshold_max_))
    {
        ROS_INFO("Failed to load z_normal_threshold_max_");
        return false;
    }
    if (!n.getParam("frontier/z_normal_min", z_normal_threshold_min_))
    {
        ROS_INFO("Failed to load z_normal_threshold_min_");
        return false;
    }
    if (!n.getParam("frontier/x_normal_max", x_normal_threshold_max_))
    {
        ROS_INFO("Failed to load x_normal_threshold_max_");
        return false;
    }
    if (!n.getParam("frontier/x_normal_min", x_normal_threshold_min_))
    {
        ROS_INFO("Failed to load x_normal_threshold_min_");
        return false;
    }
    if (!n.getParam("frontier/y_normal_max", y_normal_threshold_max_))
    {
        ROS_INFO("Failed to load y_normal_threshold_max_");
        return false;
    }
    if (!n.getParam("frontier/y_normal_min", y_normal_threshold_min_))
    {
        ROS_INFO("Failed to load y_normal_threshold_min_");
        return false;
    }
    return true;
}

bool Frontier_Processor::Initialize(const ros::NodeHandle& n, std::shared_ptr<octomap::OcTree> tree)
{
    name_ = ros::names::append(n.getNamespace(), "octomap_frontier_processor");
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }
    ros::NodeHandle nl(n);
    octomap_tree = tree;
    frontier_pub_ = nl.advertise<octomapper_frontier_processor::FrontierArray>("frontiers", 10, true);
    return true;
}

void Frontier_Processor::publishFrontiers(std::string fixed_frame_id_)
{
    ROS_INFO_STREAM("Frontier Count : "<<frontier_set_->size());
    if(publish_frontiers_ && frontier_set_->size()>5){
        frontier_pub_.publish(*tempfrontierarray_);
    }
}

bool Frontier_Processor::frontierwithinRange(const octomapper_frontier_processor::Frontier vec)
{
    for (auto odom = odom_vec.begin(); odom != odom_vec.end(); ++odom) 
    {
        if((std::sqrt((vec.x-odom->pose.pose.position.x)*(vec.x-odom->pose.pose.position.x) + 
                    (vec.y-odom->pose.pose.position.y) * (vec.y-odom->pose.pose.position.y))) < distance_within_reach)
                    return true;
    }
    return false;
}

void Frontier_Processor::updateFrontiers( bool correction,octomap::point3d lidar_point,octomap::point3d octomap_point,octomap::point3d octomap_normal)
{ //Add local and global frontiers here
    if(correction) //remove frontiers that have been covered
    {
        auto it = frontier_set_->begin();
        auto it_f = tempfrontierarray_->frontiers.begin();
        while (it !=  frontier_set_->end() && it_f != tempfrontierarray_->frontiers.end()) 
        {
            const octomapper_frontier_processor::Frontier vec = *it;
            if (octomap_tree->search(octomap::point3d(vec.x,vec.y,vec.z)) != nullptr) 
            {
                it = frontier_set_->erase(it);
                it_f = tempfrontierarray_->frontiers.erase(it_f);
                continue;
            } 
            if (frontierwithinRange(vec)) 
            {
                it = frontier_set_->erase(it);
                it_f = tempfrontierarray_->frontiers.erase(it_f);
                continue;
            } 
            else {
                ++it;
                ++it_f;
            }
        }
        odom_vec.clear();
    }
    else 
    {   
        ray_direction = (octomap_point - lidar_point).normalized();
        behind_point = octomap_point + ray_direction * octomap_tree->getResolution();
        if(boundFrontiers(behind_point,octomap_normal,lidar_point.z()))
        {
            if(octomap_tree->search(behind_point) == nullptr)
            {
            octomapper_frontier_processor::Frontier temp_frontier;
            temp_frontier.x = behind_point.x();
            temp_frontier.y = behind_point.y();
            temp_frontier.z = behind_point.z();
            temp_frontier.normal_x = -ray_direction.x();
            temp_frontier.normal_y = -ray_direction.y();
            temp_frontier.normal_z = -ray_direction.z();
            temp_frontier.free = true;
            auto insertion_result = frontier_set_->insert(temp_frontier);
            if (insertion_result.second && publish_frontiers_) {
                    tempfrontierarray_->frontiers.push_back(temp_frontier);
                }
            }
        }
    }
}

bool Frontier_Processor::boundFrontiers( octomap::point3d point, octomap::point3d normal,float lidar_height)
{
    

    return (point.x() >= octomap_tree->getBBXMin().x() && point.x() <= octomap_tree->getBBXMax().x() &&
            point.y() >= octomap_tree->getBBXMin().y() && point.y() <= octomap_tree->getBBXMax().y() &&
            point.z() >= octomap_tree->getBBXMin().z() && point.z() <= std::min(octomap_tree->getBBXMax().z(),lidar_height) &&
            normal.x() <= 0.01*x_normal_threshold_max_ && normal.x() >= -0.01*x_normal_threshold_min_ &&
            normal.y() <= 0.01*y_normal_threshold_max_ && normal.y() >= -0.01*y_normal_threshold_min_ &&
            normal.z() <= 0.01*z_normal_threshold_max_ && normal.z() >= -0.01*z_normal_threshold_min_ 
        );
}


