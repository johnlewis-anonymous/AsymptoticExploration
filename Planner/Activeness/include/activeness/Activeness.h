#pragma once
#include "ros/publisher.h"
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Odometry.h"

#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include "activeness/Viewpoint.h"
#include "activeness/ViewpointArray.h"

#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <activeness/astar_planner.hpp>




class Activeness{
    public:
        Activeness();
        ~Activeness();
        bool Initialize(const ros::NodeHandle& n);
        std::vector<ros::AsyncSpinner>  setAsynchSpinners(ros::NodeHandle& _nh);

    private:
        std::string name_;
        std::string fixed_frame_id_;
        bool viewpoint_enable_, viewpoint_publish_;
        int viewpoint_method_;
        ros::Publisher visited_viewpoints_pub_;
        std::shared_ptr<octomap::OcTree> octomap_tree_;

        bool LoadParameters(const ros::NodeHandle& _nh);
        void updateFrontiers( bool correction=true,octomap::point3d lidar_point=octomap::point3d(0,0,0),octomap::point3d octomap_point=octomap::point3d(0,0,0),octomap::point3d octomap_normal=octomap::point3d(0,0,0));
        void publishFrontiers(std::string fixed_frame_id_);
        bool boundFrontiers(octomap::point3d point, octomap::point3d normal);
        void generateViewpoints();
        void calculateVoxelCount(octomap::point3d bbx_min, octomap::point3d bbx_max);
        float updateInformationGain(activeness::Viewpoint viewpoint);
        bool distance_to_previous_goal();
        
        ros::Subscriber new_diagnostics_sub_;
        ros::CallbackQueue new_diagnostics_queue_;
        int new_diagnostics_queue_size_=1;

        activeness::Viewpoint best_viewpoint;
        activeness::ViewpointArray visited_viewpoints;

        void setDiagnostics(ros::NodeHandle& _nh);
        void NewDiagnosticsCallback(const boost::shared_ptr<const mrs_msgs::ControlManagerDiagnostics>& msg);
        bool m_currentGoalReached=false;

        ros::ServiceClient m_serviceMRSPlanner;
        ros::ServiceClient m_serviceMRSTrajectoryGenerator;
        mrs_msgs::ReferenceStampedSrv mrs_planner_srv;
        mrs_msgs::TrajectoryReferenceSrv mrs_trajectory_planner_srv;
        void publishUAVGoal(activeness::Viewpoint goal);
        void generateUAVGoal(activeness::Viewpoint goal);

        ros::CallbackQueue best_viewpoint_queue_;
        int best_viewpoint_queue_size_;
        ros::Subscriber best_viewpoint_sub_;
        void setBestViewpointSubscriber(ros::NodeHandle& _nh);
        void BestViewpointCallback(const boost::shared_ptr<const activeness::Viewpoint>& best_viewpoint);
        
        ros::CallbackQueue octomap_queue_;
        int octomap_queue_size_;
        ros::Subscriber octomap_sub_;
        void setOctomap(ros::NodeHandle& _nh);
        void NewOctomapCallback(const boost::shared_ptr<const octomap_msgs::Octomap>& msg);

        ros::CallbackQueue pose_queue_;
        int pose_queue_size_;
        ros::Subscriber pose_sub_;
        void setPose(ros::NodeHandle& _nh);
        void NewPoseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
        octomap::point3d current_odom;
        bool odom_enabled=false;


        //send trigger from activeness to viewpoint generator if the same viewpoint is recieved, if so , get the second best


        //Astar Planner
        double safe_obstacle_distance;
        double euclidean_distance_cutoff;
        double submap_distance;
        double planning_tree_resolution;
        double distance_penalty;
        double greedy_penalty;
        double timeout_threshold;
        double max_waypoint_distance;
        double min_altitude;
        double max_altitude;
        bool unknown_is_occupied;

        AstarPlanner planner;
        std::pair<std::vector<octomap::point3d>, bool> waypoints;
        octomap::point3d source,destination;
        double astar_path_distance;

        int current_state = -1;
        int previous_state = -1;
        bool destination_set;
};



