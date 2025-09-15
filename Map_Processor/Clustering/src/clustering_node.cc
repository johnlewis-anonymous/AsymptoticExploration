/*
* clustering_node.h
* Entry point for Clustering algorithms.
Authors:
  - John Lewis D (john.lewis@tecnico.ulisboa.pt)
*/

#include <clustering/Clustering.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "clustering_node");
  ros::NodeHandle n("~");

  Clustering clustering_node;
  if (!clustering_node.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize clustering node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners =
      clustering_node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
