#include <viewpoint_generator/ViewpointGenerator.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace viewpoint_generator {

class viewpoint_generatorNodelet : public nodelet::Nodelet {
 public:
  viewpoint_generatorNodelet() {}
  ~viewpoint_generatorNodelet() {}

 private:
  virtual void onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    viewpoint_generator_node_.reset(new ViewpointGenerator());
    std::vector<std::string> param_names;
    private_nh.getParamNames(param_names);
    
    
    if (!viewpoint_generator_node_->Initialize(private_nh)) {
      NODELET_ERROR("%s: Failed to initialize Viewpoint Generation nodelet.",
                    getName().c_str());
      return;
    }
    // Set asynchronous spinners
    async_spinners_ = viewpoint_generator_node_->setAsynchSpinners(private_nh);
    for (auto& spinner : async_spinners_) {
      spinner.start();
    }
    
    NODELET_INFO("Viewpoint Generation nodelet initialized successfully.");
  }

  std::unique_ptr<ViewpointGenerator> viewpoint_generator_node_;
  std::vector<ros::AsyncSpinner> async_spinners_;
};

}  

// Register nodelet
PLUGINLIB_EXPORT_CLASS(viewpoint_generator::viewpoint_generatorNodelet, nodelet::Nodelet)