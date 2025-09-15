#include "clustering/Clustering.h"
#include "clustering/FrontierArray.h"
#include <armadillo>
#include <chrono>


Clustering::Clustering()
{
    cluster_array = boost::make_shared<clustering::FrontierArray>();
}

Clustering::~Clustering() {
}

std::vector<ros::AsyncSpinner> Clustering::setAsynchSpinners(ros::NodeHandle& _nh) {
    std::vector<ros::AsyncSpinner> async_spinners;
    // Lidar spinner
    {
        ros::AsyncSpinner spinner_frontiers(1, &this->new_frontiers_queue_);
        async_spinners.push_back(spinner_frontiers);
        setFrontiers(_nh);
        ROS_INFO("[Clustering::setAsyncSpinners] : New subscriber for frontiers");
    }
return async_spinners;
}


void Clustering::setFrontiers(ros::NodeHandle& _nh) {
    ros::SubscribeOptions opts = ros::SubscribeOptions::create<clustering::FrontierArray>(
        "FRONTIERS_TOPIC",                                // topic name
        new_frontiers_queue_size_,                            // queue length
        boost::bind(&Clustering::NewFrontierCallback, this, _1), // callback
        ros::VoidPtr(),     // tracked object, we don't need one thus NULL
        &this->new_frontiers_queue_ // pointer to callback queue object
    );
    this->new_frontiers_sub_ = _nh.subscribe(opts);
}


bool Clustering::LoadParameters(const ros::NodeHandle& n)
{
    if (!n.getParam("clustering/method", clustering_method_)) //0: mlpack
    {
        ROS_INFO("Failed to load clustering_method_");
        return false;
    }
    if (!n.getParam("clustering/publish", publish_clusters_))
    {
        ROS_INFO("Failed to load publish_clusters_");
        return false;
    }
    if (!n.getParam("clustering/outlier_removal", outlier_removal_))
    {
        ROS_INFO("Failed to load outlier_removal");
        return false;
    }
    if (!n.getParam("clustering/count", cluster_count_))
    {
        ROS_INFO("Failed to load cluster_count_");
        return false;
    }
    return true;
}

bool Clustering::Initialize(const ros::NodeHandle& n)
{
    name_ = ros::names::append(n.getNamespace(), "clustering");
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }
    ros::NodeHandle nl(n);
    cluster_pub_ = nl.advertise<clustering::FrontierArray>("clusters", 10, true);
    return true;
}



void Clustering::NewFrontierCallback(const boost::shared_ptr<const clustering::FrontierArray>& msg)
{
    auto start = std::chrono::high_resolution_clock::now();
    generateClusters(msg);
    publishClusters(msg);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    ROS_INFO_STREAM("Time elapsed (microseconds) Clustering : "<<duration.count());
}



void Clustering::publishClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_)
{
    if(publish_clusters_)
    {
        cluster_array->frontiers.clear();
        int index = 0;
        if(clustering_method_ == 0) //Kmeans
            {
                for (auto it = frontier_set_->frontiers.begin(); it != frontier_set_->frontiers.end(); ++it,++index) 
                {
                    clustering::Frontier vec = *it;   
                    vec.max_cluster_count = assignments.max()+1;
                    vec.cluster_centroid_x = centroids.col(assignments[index])[0];
                    vec.cluster_centroid_y = centroids.col(assignments[index])[1];
                    vec.cluster_centroid_z = centroids.col(assignments[index])[2];
                    vec.cluster_number = assignments.at(index);
                    cluster_array->frontiers.push_back(vec);
                }
            }
        else if(clustering_method_ == 2) //Mean shift Kmeans
            {
                for (auto it = frontier_set_->frontiers.begin(); it != frontier_set_->frontiers.end(); ++it,++index) 
                {
                    clustering::Frontier vec = *it;   
                    vec.max_cluster_count = assignments.max()+1;
                    vec.cluster_number = assignments.at(index);
                    vec.cluster_centroid_x = centroids.col(assignments[index])[0];
                    vec.cluster_centroid_y = centroids.col(assignments[index])[1];
                    vec.cluster_centroid_z = centroids.col(assignments[index])[2];
                    cluster_array->frontiers.push_back(vec);
                }
            }
        cluster_pub_.publish(*cluster_array);
    }
}

void Clustering::generateClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_)
{
    if(clustering_method_ == 0)
    {
        generateKmeansClusters(frontier_set_);
    }
    else if(clustering_method_ == 2){
        generateMeanshiftClusters(frontier_set_);
    }
    
}
void Clustering::generateMeanshiftClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_)
{
    // arma::mat frontier_arma(2, frontier_set_->frontiers.size());
    arma::mat frontier_arma(3, frontier_set_->frontiers.size());
    int index = 0;
    for (auto it = frontier_set_->frontiers.begin(); it != frontier_set_->frontiers.end(); ++it, ++index) 
    {
        frontier_arma(0, index) = it->x ; 
        frontier_arma(1, index) = it->y ; 
        frontier_arma(2, index) = it->z ; 
    }
    assignments.reset();
    if(centroids.size()>0)
        meanshift.Cluster(frontier_arma, assignments,centroids,false,true);
    else
        meanshift.Cluster(frontier_arma, assignments,centroids,false,false);
}

void Clustering::generateKmeansClusters(const boost::shared_ptr<const clustering::FrontierArray>& frontier_set_)
{
    // arma::mat frontier_arma(2, frontier_set_->frontiers.size());
    arma::mat frontier_arma(3, frontier_set_->frontiers.size());
    int index = 0;
    for (auto it = frontier_set_->frontiers.begin(); it != frontier_set_->frontiers.end(); ++it, ++index) 
    {
        frontier_arma(0, index) = it->x ; 
        frontier_arma(1, index) = it->y ; 
        frontier_arma(2, index) = it->z ; 
    }
    assignments.reset();
    if(centroids.size()>0)
        kmeans.Cluster(frontier_arma, cluster_count_, assignments,centroids,false,true);
    else
        kmeans.Cluster(frontier_arma, cluster_count_, assignments,centroids,false,false);
}
