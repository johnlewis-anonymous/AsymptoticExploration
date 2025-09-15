#include "viewpoint_generator/ViewpointGenerator.h"
#include "nav_msgs/Odometry.h"
#include "octomap/octomap_types.h"
#include "octomap_msgs/Octomap.h"
#include "ros/console.h"
#include "viewpoint_generator/Frontier.h"
#include "viewpoint_generator/FrontierArray.h"
#include "viewpoint_generator/Viewpoint.h"
#include "viewpoint_generator/ViewpointArray.h"
#include <algorithm>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <cmath>


ViewpointGenerator::ViewpointGenerator()
{
    viewpoint_array = boost::make_shared<viewpoint_generator::ViewpointArray>();
}

ViewpointGenerator::~ViewpointGenerator() {
}

bool ViewpointGenerator::LoadParameters(const ros::NodeHandle& n)
{
    
    if (!n.getParam("viewpoint/publish", viewpoint_publish_))
        {
        ROS_INFO("Failed to load viewpoint_publish_");
        return false;
        }
    if (!n.getParam("viewpoint/method", viewpoint_method_))
        {
        ROS_INFO("Failed to load viewpoint_method_");
        return false;
        }
    if (!n.getParam("viewpoint/box_length", inf_gain_cube_side))
        {
        ROS_INFO("Failed to load viewpoint_box_length_");
        return false;
        }
    if (!n.getParam("viewpoint/k_gain", k_gain))
        {
        ROS_INFO("Failed to load viewpoint_k_gain");
        return false;
        }
    if (!n.getParam("viewpoint/lambda", lambda))
        {
        ROS_INFO("Failed to load viewpoint_lambda");
        return false;
        }
    if (!n.getParam("viewpoint/bucket_distance", viewpoint_bucket_distance))
        {
        ROS_INFO("Failed to load viewpoint_bucket_distance");
        return false;
        }
        
    return true;
}

std::vector<ros::AsyncSpinner> ViewpointGenerator::setAsynchSpinners(ros::NodeHandle& _nh) {
    std::vector<ros::AsyncSpinner> async_spinners;
    // pose spinner
    {
        ros::AsyncSpinner spinner_visited_viewpoints(1, &this->new_visited_viewpoints_queue_);
        async_spinners.push_back(spinner_visited_viewpoints);
        setVisitedViewpoints(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for visited viewpoints");
    }
    // pose spinner
    {
        ros::AsyncSpinner spinner_pose(1, &this->new_pose_queue_);
        async_spinners.push_back(spinner_pose);
        setPose(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for pose");
    }
    // Cluster spinner
    {
        ros::AsyncSpinner spinner_cluster(1, &this->new_cluster_queue_);
        async_spinners.push_back(spinner_cluster);
        setCluster(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for cluster");
    }
    {
        ros::AsyncSpinner spinner_octomap(1, &this->new_octomap_queue_);
        async_spinners.push_back(spinner_octomap);
        setOctomap(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for octomap");
    }
return async_spinners;
}

void ViewpointGenerator::setVisitedViewpoints(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<viewpoint_generator::ViewpointArray>(
        "VISITED_VIEWPOINTS_TOPIC",                                // topic name
        new_visited_viewpoints_queue_size_,                            // queue length
        boost::bind(&ViewpointGenerator::NewVisitedViewpointsCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_visited_viewpoints_queue_ // pointer to callback queue object
    );
    this->new_visited_viewpoints_sub_ = _nh.subscribe(opts);
}

void ViewpointGenerator::setOctomap(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<octomap_msgs::Octomap>(
        "OCTOMAP_TOPIC",                                // topic name
        new_octomap_queue_size_,                            // queue length
        boost::bind(&ViewpointGenerator::NewOctomapCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_octomap_queue_ // pointer to callback queue object
    );
    this->new_octomap_sub_ = _nh.subscribe(opts);
}


void ViewpointGenerator::setPose(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        "POSE_TOPIC",                                // topic name
        new_pose_queue_size_,                            // queue length
        boost::bind(&ViewpointGenerator::NewPoseCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_pose_queue_ // pointer to callback queue object
    );
    this->new_pose_sub_ = _nh.subscribe(opts);
}

void ViewpointGenerator::setCluster(ros::NodeHandle& _nh) {
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<viewpoint_generator::FrontierArray>(
        "CLUSTER_TOPIC",                                // topic name
        new_cluster_queue_size_,                            // queue length
        boost::bind(&ViewpointGenerator::NewClusterCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_cluster_queue_ // pointer to callback queue object
    );
    this->new_cluster_sub_ = _nh.subscribe(opts);
}

bool ViewpointGenerator::Initialize(const ros::NodeHandle& n)
{
    name_ = ros::names::append(n.getNamespace(), "viewpoint_generator");
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }
    ros::NodeHandle nl(n);
    viewpoints_pub_ = nl.advertise<viewpoint_generator::ViewpointArray>("viewpoints", 10, true);
    best_viewpoint_pub_ = nl.advertise<viewpoint_generator::Viewpoint>("best_view", 10, true);
    return true;
}

void ViewpointGenerator::NewVisitedViewpointsCallback(const viewpoint_generator::ViewpointArray::ConstPtr& msg){
    if(msg)
    {
        visited_viewpoints = *msg;
    }

}

void ViewpointGenerator::NewOctomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    // Convert the message to an AbstractOcTree
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(*msg);
    
    // Attempt to cast to OcTree
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(abstract_tree));

    this->octomap_tree_ = octree;
    
    if(octree)
    {
        octree_enabled=true;
        bounding_box_voxel_count = (inf_gain_cube_side*inf_gain_cube_side*inf_gain_cube_side)/(octomap_tree_->getResolution()*octomap_tree_->getResolution()*octomap_tree_->getResolution());
    }
}


void ViewpointGenerator::NewPoseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    if(pose_msg)
    {
        current_odom = *pose_msg;
        odom_enabled=true;
    }
}

void ViewpointGenerator::NewClusterCallback(const boost::shared_ptr<const viewpoint_generator::FrontierArray>& cluster_array)
{
    if(!cluster_array->frontiers.empty())
        {
            {
                auto start = std::chrono::high_resolution_clock::now();
                
                setViewpoints(cluster_array);
                viewpoints_pub_.publish(viewpoint_array);
                if(chosen_viewpoint_set)
                {
                    best_viewpoint_pub_.publish(chosen_viewpoint);
                }
                else {
                {
                    ROS_INFO_STREAM("Failed to publish");
                }
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                ROS_INFO_STREAM("Time elapsed (microseconds) ViewpointGeneration + Best Viewpoint : "<<duration.count());
            }
        }
}

void ViewpointGenerator::setViewpoints(viewpoint_generator::FrontierArray::ConstPtr cluster_array)
{
    viewpoint_array->viewpoints.clear();
    cluster_check.clear();
    cluster_check.resize(cluster_array->frontiers.at(0).max_cluster_count+1,false);    
    
    if(!octree_enabled || !odom_enabled )
    {
        return;
    }
    //the following code checks for centroids
    max_inf_gain=-100000;
    chosen_viewpoint_set = false;
    for (auto it = cluster_array->frontiers.begin(); it != cluster_array->frontiers.end(); ++it) 
    {
        if(!cluster_check.at(it->cluster_number) && checkPreviousViewpoints(*it))
        {
            temporary_viewpoint.x = it->cluster_centroid_x;
            temporary_viewpoint.y = it->cluster_centroid_y;
            temporary_viewpoint.z = current_odom.pose.pose.position.z;
            temporary_viewpoint.cluster_number = it->cluster_number;
            temporary_viewpoint.max_cluster_count = it->max_cluster_count;
            //bound height here after getting some
            temporary_viewpoint.distance = updateDistance(it->cluster_centroid_x,it->cluster_centroid_y,current_odom.pose.pose.position.z,current_odom);
            calculateVoxelCount(octomap::point3d(temporary_viewpoint.x,temporary_viewpoint.y,current_odom.pose.pose.position.z)- octomap::point3d(inf_gain_cube_side/2,inf_gain_cube_side/2,inf_gain_cube_side/2), 
            octomap::point3d(temporary_viewpoint.x,temporary_viewpoint.y,current_odom.pose.pose.position.z)+ octomap::point3d(inf_gain_cube_side/2,inf_gain_cube_side/2,inf_gain_cube_side/2));
            temporary_viewpoint.free_cell_count = temporary_voxel_count[0];//0 free 
            temporary_viewpoint.occupied_cell_count = temporary_voxel_count[1];//1 occupied  
            temporary_viewpoint.unknown_cell_count = temporary_voxel_count[2];//2 unknown
            temporary_viewpoint.information_gain =  updateInformationGain(temporary_viewpoint);
            viewpoint_array->viewpoints.push_back(temporary_viewpoint);
            cluster_check.at(it->cluster_number)=true;
            if(temporary_viewpoint.information_gain>max_inf_gain)
            {
                chosen_viewpoint = temporary_viewpoint;
                chosen_viewpoint_set = true;
                max_inf_gain = temporary_viewpoint.information_gain;
            }
        }
    }
    if(!chosen_viewpoint_set)
    {
        ROS_INFO("Viewpoint not set");
        ROS_INFO_STREAM(cluster_array->frontiers.size());
    }
}

bool ViewpointGenerator::checkPreviousViewpoints(viewpoint_generator::Frontier temporary_viewpoint)
{
    for (auto viewpt = visited_viewpoints.viewpoints.begin(); viewpt != visited_viewpoints.viewpoints.end(); ++viewpt) 
    {
        if(std::sqrt((temporary_viewpoint.x-viewpt->x)*(temporary_viewpoint.x-viewpt->x) + 
                    (temporary_viewpoint.y-viewpt->y) * (temporary_viewpoint.y-viewpt->y) + 
                    (temporary_viewpoint.z-viewpt->z) * (temporary_viewpoint.z-viewpt->z)) < viewpoint_bucket_distance)
                    {
                        return false;
                    }
    }
    if(updateDistance(temporary_viewpoint.x, temporary_viewpoint.y, temporary_viewpoint.z,current_odom)<2)
        return false;
    return true;
}


float ViewpointGenerator::updateInformationGain(viewpoint_generator::Viewpoint viewpoint)
{
    return k_gain * static_cast<float>(viewpoint.unknown_cell_count)/bounding_box_voxel_count * exp(- lambda * viewpoint.distance);
}

void ViewpointGenerator::calculateVoxelCount(octomap::point3d bbxMin, octomap::point3d bbxMax)
{
    bbxMin = boundCube(bbxMin,false);
    bbxMax = boundCube(bbxMax,true);
    temporary_voxel_count[0]=0;//0 free 
    temporary_voxel_count[1]=0;//1 occupied  
    temporary_voxel_count[2]=0;//2 unknown
    for (auto it = octomap_tree_->begin_leafs_bbx(bbxMin, bbxMax),end = octomap_tree_->end_leafs_bbx(); it != end; ++it) 
    {
        if (octomap_tree_->isNodeOccupied(*it))
            temporary_voxel_count[1]++;
        else
            temporary_voxel_count[0]++;
    }
    temporary_voxel_count[2] = bounding_box_voxel_count - (temporary_voxel_count[1] + temporary_voxel_count[0]);
}

float ViewpointGenerator::updateDistance(float x, float y, float z,nav_msgs::Odometry odom)
{
    return std::sqrt((odom.pose.pose.position.x-x)*(odom.pose.pose.position.x-x) + 
                        (odom.pose.pose.position.y-y) * (odom.pose.pose.position.y-y) + 
                        (odom.pose.pose.position.z-z) * (odom.pose.pose.position.z-z));
}

octomap::point3d ViewpointGenerator::boundCube(octomap::point3d point, bool max)
{
    if(!max)
    {
        point.x() = std::max(octomap_tree_->getBBXMin().x(),point.x());
        point.y() = std::max(octomap_tree_->getBBXMin().y(),point.y());
        point.z() = std::max(octomap_tree_->getBBXMin().z(),point.z());
    }
    else {
        point.x() = std::min(octomap_tree_->getBBXMax().x(),point.x());
        point.y() = std::min(octomap_tree_->getBBXMax().y(),point.y());
        point.z() = std::min(octomap_tree_->getBBXMax().z(),point.z());    
    }
    return point;
}