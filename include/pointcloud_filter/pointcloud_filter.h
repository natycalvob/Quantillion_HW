#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudFilter {
public:
  // Parametrized constructor
  PointCloudFilter(int throttle_rate, float intensity_threshold);

  // Destructor
  ~PointCloudFilter();

  // Callback function for PointCloud messages
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;

  int throttle_rate_;       // Throttle rate to skip messages
  int intensity_threshold_; // Intensity threshold for filtering
};
