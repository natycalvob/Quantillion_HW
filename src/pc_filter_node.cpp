#include "pointcloud_filter/pointcloud_filter.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "pointcloud_filter_node_hw");
  ROS_INFO("Initliazing ROS Node");

  // Default values
  int throttle_rate = 2;
  float intensity_threshold = 1000.0;

  // Initialize ROS node handle
  ros::NodeHandle nh("~");

  // Load parameters from the ROS parameter server or launch file
  nh.param<int>("throttle_rate", throttle_rate, 2);
  nh.param<float>("intensity_threshold", intensity_threshold, 1000.0);

  // Create an instance of PointCloudFilter with specified parameters
  PointCloudFilter filter(throttle_rate, intensity_threshold);

  // Start processing incoming ROS messages
  ros::spin();

  return 0;
}