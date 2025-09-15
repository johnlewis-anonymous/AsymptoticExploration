#include "activeness/Activeness.h"
#include "activeness/Viewpoint.h"
#include <activeness/astar_planner.hpp>
#include "mrs_msgs/ControlManagerDiagnostics.h"
#include "mrs_msgs/TrajectoryReferenceSrv.h"
#include "octomap/octomap_types.h"
#include "ros/console.h"
#include "ros/time.h"
#include <cmath>


Activeness::Activeness()
{
    destination_set=false;
}

Activeness::~Activeness() {
}


bool Activeness::Initialize(const ros::NodeHandle& n)
{
    name_ = ros::names::append(n.getNamespace(), "activeness");
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }
    if (!planner.LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load astar parameters.", name_.c_str());
        return false;
    }
    ros::NodeHandle nl(n);
    m_serviceMRSPlanner = nl.serviceClient<mrs_msgs::ReferenceStampedSrv>("PLANNER_SERVICE");
    m_serviceMRSTrajectoryGenerator = nl.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("TRAJECTORY_SERVICE");
    visited_viewpoints_pub_ = nl.advertise<activeness::ViewpointArray>("visited_viewpoints", 10, true);
    return true;
}


bool Activeness::LoadParameters(const ros::NodeHandle& n)
{
    if (!n.getParam("frame_id/fixed", fixed_frame_id_))
        {
        ROS_INFO("Failed to load fixed_frame_id_");
        return false;
        }
    return true;
}

std::vector<ros::AsyncSpinner> Activeness::setAsynchSpinners(ros::NodeHandle& _nh) {
    std::vector<ros::AsyncSpinner> async_spinners;
    // pose spinner
    {
        ros::AsyncSpinner spinner_pose(1, &this->pose_queue_);
        async_spinners.push_back(spinner_pose);
        setPose(_nh);
        ROS_INFO("[ViewpointGeneration::setAsyncSpinners] : New subscriber for pose");
    }
    // diagnostics spinner
    {
        ros::AsyncSpinner spinner_diagnostics(1, &this->new_diagnostics_queue_);
        async_spinners.push_back(spinner_diagnostics);
        setDiagnostics(_nh);
        ROS_INFO("[Activeness::setAsyncSpinners] : New subscriber for Diagnostics");
    }
    // viewpoint  spinner
    {
    ros::AsyncSpinner spinner_best_viewpoint(1, &this->best_viewpoint_queue_);
    async_spinners.push_back(spinner_best_viewpoint);
    setBestViewpointSubscriber(_nh);
    ROS_INFO("[Activeness::setAsyncSpinners] : New subscriber for BestViewpoint");
    }
    // octomap  spinner
    {
    ros::AsyncSpinner spinner_octomap(1, &this->octomap_queue_);
    async_spinners.push_back(spinner_octomap);
    setOctomap(_nh);
    }
return async_spinners;
}

void Activeness::setPose(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        "POSE_TOPIC",                                // topic name
        pose_queue_size_,                            // queue length
        boost::bind(&Activeness::NewPoseCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->pose_queue_ // pointer to callback queue object
    );
    this->pose_sub_ = _nh.subscribe(opts);
}


void Activeness::setOctomap(ros::NodeHandle& _nh) {
  // Create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts = ros::SubscribeOptions::create<octomap_msgs::Octomap>(
      "OCTOMAP_TOPIC",                                // topic name
      octomap_queue_size_,                            // queue length
      boost::bind(&Activeness::NewOctomapCallback, this, _1), // callback
      ros::VoidPtr(),     // tracked object, we don't need one thus NULL
      &this->octomap_queue_ // pointer to callback queue object
  );
  this->octomap_sub_ = _nh.subscribe(opts);
}


void Activeness::setDiagnostics(ros::NodeHandle& _nh){
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<mrs_msgs::ControlManagerDiagnostics>(
        "DIAGNOSTICS_TOPIC",                                // topic name
        new_diagnostics_queue_size_,                            // queue length
        boost::bind(&Activeness::NewDiagnosticsCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_diagnostics_queue_ // pointer to callback queue object
    );
    this->new_diagnostics_sub_ = _nh.subscribe(opts);
}

void Activeness::setBestViewpointSubscriber(ros::NodeHandle& _nh) {
  // Create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts =
      ros::SubscribeOptions::create<activeness::Viewpoint>(
          "BEST_VIEWPOINT_TOPIC",                                // topic name
          best_viewpoint_queue_size_,                                // queue length
          boost::bind(&Activeness::BestViewpointCallback, this, _1), // callback
          ros::VoidPtr(),    // tracked object, we don't need one thus NULL
          &this->best_viewpoint_queue_ // pointer to callback queue object
      );
  this->best_viewpoint_sub_ = _nh.subscribe(opts);
  
}


void Activeness::NewPoseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    if(pose_msg)
    {
        current_odom = octomap::point3d(pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,pose_msg->pose.pose.position.z);
        odom_enabled=true;
    }
}

void Activeness::NewDiagnosticsCallback(const boost::shared_ptr<const mrs_msgs::ControlManagerDiagnostics>& msg)
{
    // astar_path_distance = 0.0;
    // if(!msg->tracker_status.have_goal && odom_enabled && viewpoint_enable_ && this->octomap_tree_) 
    // {
    //     ROS_INFO_STREAM("---------------------------------------------First");
    //     source = current_odom;
    //     destination = octomap::point3d(best_viewpoint.x,best_viewpoint.y,best_viewpoint.z);
    //     waypoints = planner.findPath(source, destination,octomap_tree_, timeout_threshold);
    //     for (size_t i = 1; i < waypoints.first.size(); i++) {
    //         astar_path_distance += waypoints.first[i-1].distance(waypoints.first[i]);
    //     }
    //     ROS_INFO_STREAM(astar_path_distance);
    // }
    // else if(msg->tracker_status.have_goal && viewpoint_enable_ && this->octomap_tree_ && distance_to_previous_goal()) { 
    //     ROS_INFO_STREAM("---------------------------------------------Enabled");
    //     source = destination;
    //     destination = octomap::point3d(best_viewpoint.x,best_viewpoint.y,best_viewpoint.z);
    //     waypoints = planner.findPath(source, destination,octomap_tree_, timeout_threshold);
    //     for (size_t i = 1; i < waypoints.first.size(); i++) {
    //         astar_path_distance += waypoints.first[i-1].distance(waypoints.first[i]);
    //     }
    //     ROS_INFO_STREAM(astar_path_distance);
    // }
    // else if(!distance_to_previous_goal())
    // {
    //     ROS_INFO("Waiting to reach goal");
    // }
    // else {
    //     ROS_INFO("One of these failed");
    //     ROS_INFO("Tracker status - have_goal: %s, odom_enabled: %s, viewpoint_enable: %s, octomap_tree: %p", 
    //             msg->tracker_status.have_goal ? "true" : "false",
    //             odom_enabled ? "true" : "false", 
    //             viewpoint_enable_ ? "true" : "false",
    //             this->octomap_tree_.get());
    // }
    // if(astar_path_distance>0)
    // {
    //     // m_serviceMRSTrajectoryGenerator
    //     // mrs_trajectory_planner_srv
    //     mrs_trajectory_planner_srv.request.trajectory.points.clear();
    //     mrs_trajectory_planner_srv.request.trajectory.header.stamp = ros::Time::now();
    //     mrs_trajectory_planner_srv.request.trajectory.input_id = 1;
    //     mrs_trajectory_planner_srv.request.trajectory.use_heading = true;
    //     mrs_trajectory_planner_srv.request.trajectory.fly_now = true;
    //     mrs_trajectory_planner_srv.request.trajectory.loop = false;
    //     mrs_trajectory_planner_srv.request.trajectory.dt = 0.2;

    //     for (size_t i = 0; i < waypoints.first.size(); i++) {
    //         mrs_msgs::Reference ref;
    //         ref.position.x = waypoints.first[i-1].x();
    //         ref.position.y = waypoints.first[i-1].y();
    //         ref.position.z = waypoints.first[i-1].z();
    //         ref.heading = 0;
    //         mrs_trajectory_planner_srv.request.trajectory.points.push_back(ref);
    //     }
    // if (m_serviceMRSTrajectoryGenerator.call(mrs_trajectory_planner_srv)) {
    //     ROS_INFO("New trajectory accepted");
    // }
    // else {
    //     ROS_ERROR("Problem with your trajectory");
    // }
    // }
    // else {
    //     ROS_WARN("Astar issue bruh");
    // }
        
    if((distance_to_previous_goal() || !destination_set || !msg->tracker_status.have_goal) && viewpoint_enable_) 
    {
        publishUAVGoal(best_viewpoint);
    }
    
}

bool Activeness::distance_to_previous_goal()
{
    ROS_INFO_STREAM("Distance to goal" << current_odom.distance(destination));
    ROS_INFO_STREAM("Goal" << destination);
    return current_odom.distance(destination) < 2;
}

void Activeness::NewOctomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    // Convert the message to an AbstractOcTree
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(*msg);
    
    // Attempt to cast to OcTree
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(abstract_tree));

    this->octomap_tree_ = octree;
    
}


void Activeness::BestViewpointCallback(const boost::shared_ptr<const activeness::Viewpoint>& best_viewpoint)
{
    if(best_viewpoint)    
        {
        this->best_viewpoint = *best_viewpoint;
        viewpoint_enable_ = true;
        }
}




void Activeness::generateUAVGoal(activeness::Viewpoint goal)
	{
        
        source = current_odom;
        destination = octomap::point3d(goal.x,goal.y,goal.z);
        
        waypoints = planner.findPath(source, destination, octomap_tree_, timeout_threshold);
        double total_distance = 0.0;
        
	}


void Activeness::publishUAVGoal(activeness::Viewpoint goal)
	{
		geometry_msgs::PoseStamped m_goal;
		m_goal.header.frame_id = fixed_frame_id_;
		m_goal.header.stamp = ros::Time::now();

		m_goal.pose.position.x = goal.x;
		m_goal.pose.position.y = goal.y;
		m_goal.pose.position.z = goal.z;
		m_goal.pose.orientation.x = 0;
		m_goal.pose.orientation.y = 0;
		m_goal.pose.orientation.z = 0;
		m_goal.pose.orientation.w = 1;

		mrs_planner_srv.request.header = m_goal.header;
		mrs_planner_srv.request.reference.position = m_goal.pose.position;
		mrs_planner_srv.request.reference.heading = 0.0; 
        
        ROS_INFO("Entering Service call.");
        
		if (m_serviceMRSPlanner.call(mrs_planner_srv)) {
        ROS_INFO("Service call succeeded.");
        destination_set = true;
        destination = octomap::point3d(goal.x,goal.y,goal.z);
		ROS_INFO_STREAM(goal.x << " " << goal.y << " " << goal.z << " -> Goal published to service");
        visited_viewpoints.viewpoints.push_back(goal);
        visited_viewpoints_pub_.publish(visited_viewpoints);
		} else {
			ROS_ERROR("Failed to call service");
		}
	}
