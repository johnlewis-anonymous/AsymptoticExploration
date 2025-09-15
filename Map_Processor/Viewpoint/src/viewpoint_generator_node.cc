/*
* octomapper_frontier.h
* Entry point for the octomapper_frontier generation ROS node.
Authors:
  - John Lewis D (john.lewis@tecnico.ulisboa.pt)
*/

#include <viewpoint_generator/ViewpointGenerator.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "viewpoint_generator");
  ros::NodeHandle n("~");

  ViewpointGenerator viewpoint_generator_node;
  if (!viewpoint_generator_node.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize viewpoint_generator node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners =
      viewpoint_generator_node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
