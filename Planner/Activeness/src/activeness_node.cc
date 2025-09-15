/*
* octomapper_frontier.h
* Entry point for the octomapper_frontier generation ROS node.
Authors:
  - John Lewis D (john.lewis@tecnico.ulisboa.pt)
*/

#include <activeness/Activeness.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "activeness");
  ros::NodeHandle n("~");

  Activeness activeness_node;
  if (!activeness_node.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize activeness node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners =
      activeness_node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
