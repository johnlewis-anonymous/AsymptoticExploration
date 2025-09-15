#pragma once

#include "mlpack/methods/kmeans/kmeans.hpp"
#include "mlpack/methods/mean_shift/mean_shift.hpp"
#include "armadillo"

#include "ros/publisher.h"
#include <ros/subscribe_options.h>
#include "ros/callback_queue.h"
#include <ros/ros.h>


#include <memory>
#include <ros/ros.h>
#include <clustering/TypeDefs.h>
#include <string>
#include <vector>

#include <tf/transform_listener.h>

#include <clustering/Frontier.h>
#include <clustering/FrontierArray.h>


class Clustering{
    public:
        Clustering();
        ~Clustering();
        bool Initialize(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& _nh);
        std::vector<ros::AsyncSpinner> setAsynchSpinners(ros::NodeHandle& _nh);
    
    private:
        std::string name_;
        int clustering_method_;
        clustering::FrontierArray::Ptr cluster_array;
        clustering::Frontier temp_cluster_;
        
        ros::CallbackQueue new_frontiers_queue_;
        int new_frontiers_queue_size_=1;
        bool publish_clusters_,publish_viewpoints_,outlier_removal_, viewpoint_enable_;
        double viewpoint_sampling_distance_min_,viewpoint_sampling_distance_max_;
        ros::Publisher cluster_pub_, view_point_pub_;
        void generateClusters(const boost::shared_ptr<const clustering::FrontierArray>& msg);
        
        void generateViewpoints();
        void octreeLookuptable();

        mlpack::kmeans::KMeans<> kmeans;
        mlpack::meanshift::MeanShift<> meanshift;
        arma::Row<size_t> assignments;

        arma::mat centroids;
        int cluster_count_;
        double distance_from_viewpoint_;
        int discrete_points;
        void setFrontiers(ros::NodeHandle& _nh);
        void setOctomap(ros::NodeHandle& _nh);
        void NewFrontierCallback(const boost::shared_ptr<const clustering::FrontierArray>& msg);
        
        ros::Subscriber new_frontiers_sub_,new_octomap_sub_;
        void publishClusters(const boost::shared_ptr<const clustering::FrontierArray>& msg);
        void generateKmeansClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_);
        void generateMeanshiftClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_);

        std::vector<int> cluster_labels;
        std::vector<int> getClusterLabels() const {
            return cluster_labels;
        }
};



