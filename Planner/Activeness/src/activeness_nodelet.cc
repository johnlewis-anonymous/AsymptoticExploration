#include <activeness/Activeness.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace activeness {

class activenessNodelet : public nodelet::Nodelet {
 public:
  activenessNodelet() {}
  ~activenessNodelet() {}

 private:
  virtual void onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    activeness_node_.reset(new Activeness());
    std::vector<std::string> param_names;
    private_nh.getParamNames(param_names);
    
    
    if (!activeness_node_->Initialize(private_nh)) {
      NODELET_ERROR("%s: Failed to initialize Viewpoint Generation nodelet.",
                    getName().c_str());
      return;
    }
    // Set asynchronous spinners
    async_spinners_ = activeness_node_->setAsynchSpinners(private_nh);
    for (auto& spinner : async_spinners_) {
      spinner.start();
    }
    
    NODELET_INFO("Viewpoint Generation nodelet initialized successfully.");
  }

  std::unique_ptr<Activeness> activeness_node_;
  std::vector<ros::AsyncSpinner> async_spinners_;
};

}  

// Register nodelet
PLUGINLIB_EXPORT_CLASS(activeness::activenessNodelet, nodelet::Nodelet)