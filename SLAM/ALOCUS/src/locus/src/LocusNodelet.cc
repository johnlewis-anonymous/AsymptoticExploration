#include <locus/Locus.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace locus {

class LocusNodelet : public nodelet::Nodelet {
 public:
  LocusNodelet() {}
  ~LocusNodelet() {}

 private:
  virtual void onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    
    
    locus_node_.reset(new Locus());
    std::vector<std::string> param_names;
    private_nh.getParamNames(param_names);
    // for (const auto& param : param_names) {
    //   NODELET_INFO("  - %s", param.c_str());
    // }
    
    
    if (!locus_node_->Initialize(private_nh, false)) {
      NODELET_ERROR("%s: Failed to initialize locus nodelet.",
                    getName().c_str());
      return;
    }
    NODELET_INFO("Locus Hello2");
    // Set asynchronous spinners
    async_spinners_ = locus_node_->setAsynchSpinners(private_nh);
    for (auto& spinner : async_spinners_) {
      spinner.start();
    }
    
    NODELET_INFO("Locus nodelet initialized successfully.");
  }

  std::unique_ptr<Locus> locus_node_;
  std::vector<ros::AsyncSpinner> async_spinners_;
};

}  // namespace locus

// Register nodelet
PLUGINLIB_EXPORT_CLASS(locus::LocusNodelet, nodelet::Nodelet)