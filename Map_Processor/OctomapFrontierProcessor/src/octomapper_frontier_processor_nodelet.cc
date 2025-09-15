#include <octomapfrontierprocessor/Octomapper.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace octomapper_frontier_processor {

class octomapper_frontier_processorNodelet : public nodelet::Nodelet {
 public:
  octomapper_frontier_processorNodelet() {}
  ~octomapper_frontier_processorNodelet() {}

 private:
  virtual void onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    octomap_frontier_processor_node_.reset(new Octomapper());
    std::vector<std::string> param_names;
    private_nh.getParamNames(param_names);
    
    
    if (!octomap_frontier_processor_node_->Initialize(private_nh)) {
      NODELET_ERROR("%s: Failed to initialize Octomapper_Frontier nodelet.",
                    getName().c_str());
      return;
    }
    // Set asynchronous spinners
    async_spinners_ = octomap_frontier_processor_node_->setAsynchSpinners(private_nh);
    for (auto& spinner : async_spinners_) {
      spinner.start();
    }
    
    NODELET_INFO("Octomapper_Frontier nodelet initialized successfully.");
  }

  std::unique_ptr<Octomapper> octomap_frontier_processor_node_;
  std::vector<ros::AsyncSpinner> async_spinners_;
};

}  

// Register nodelet
PLUGINLIB_EXPORT_CLASS(octomapper_frontier_processor::octomapper_frontier_processorNodelet, nodelet::Nodelet)