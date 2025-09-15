/*
* octomapper_frontier.h
* Entry point for the octomapper_frontier generation ROS node.
Authors:
  - John Lewis D (john.lewis@tecnico.ulisboa.pt)
*/

#include <octomapfrontierprocessor/Octomapper.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomapper_frontier_processor");
  ros::NodeHandle n("~");

  Octomapper octomapper_frontier_processor_node;
  if (!octomapper_frontier_processor_node.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize octomapper_frontier_processor node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners =
      octomapper_frontier_processor_node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
