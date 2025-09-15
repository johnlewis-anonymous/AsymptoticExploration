#include <clustering/Clustering.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace clustering {

class ClusteringNodelet : public nodelet::Nodelet {
 public:
  ClusteringNodelet() {}
  ~ClusteringNodelet() {}

 private:
  virtual void onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    clustering_node_.reset(new Clustering());
    NODELET_INFO("Private namespace: %s", private_nh.getNamespace().c_str());
    NODELET_INFO("Public namespace: %s", nh.getNamespace().c_str());
    std::vector<std::string> param_names;
    private_nh.getParamNames(param_names);
    NODELET_INFO("Found %zu parameters in private namespace:", param_names.size());
    
    if (!clustering_node_->Initialize(private_nh)) {
      NODELET_ERROR("%s: Failed to initialize clustering nodelet.",
                    getName().c_str());
      return;
    }
    // Set asynchronous spinners
    async_spinners_ = clustering_node_->setAsynchSpinners(private_nh);
    for (auto& spinner : async_spinners_) {
      spinner.start();
    }
    
    NODELET_INFO("Clustering nodelet initialized successfully.");
  }

  std::unique_ptr<Clustering> clustering_node_;
  std::vector<ros::AsyncSpinner> async_spinners_;
};

}  // namespace locus

// Register nodelet
PLUGINLIB_EXPORT_CLASS(clustering::ClusteringNodelet, nodelet::Nodelet)