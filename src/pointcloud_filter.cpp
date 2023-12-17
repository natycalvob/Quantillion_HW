#include "pointcloud_filter/pointcloud_filter.h"
#include <pcl_conversions/pcl_conversions.h> // Include PCL conversions header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Constructor - Initializes the subscriber and publisher
PointCloudFilter::PointCloudFilter(int throttle_rate, float intensity_threshold)
    : nh_("~"), throttle_rate_(throttle_rate),
      intensity_threshold_(intensity_threshold) {

  // Subscribe to the input PointCloud topic
  ROS_INFO("Subscribing PointCloud2 raw data");
  pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      "/input_pointcloud", 1, &PointCloudFilter::pointCloudCallback, this);

  // Advertise the filtered PointCloud topic
  ROS_INFO("Publishing PointCloud2 filtered data");
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
}

// Destructor
PointCloudFilter::~PointCloudFilter() {}

// Callback function triggered upon receiving a PointCloud2 message
void PointCloudFilter::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {

  ROS_INFO("Received a PointCloud message.");
  static int count = 0; // Counter to implement message throttling

  // Throttle the messages based on throttle_rate_
  if (count % throttle_rate_ == 0) {
    ROS_INFO("Received a PointCloud message. Filtering...");

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Iterate through points and apply intensity thresholding
    for (auto &point : pcl_cloud->points) {
      if (point.intensity <= intensity_threshold_) {
        // Remove the point by setting its coordinates to NaN
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue; // Skip unnecessary else block
      }
      // Keep the point
    }

    ROS_INFO("Filtering completed.");

    // Convert back to ROS PointCloud2
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*pcl_cloud, filtered_msg);

    ROS_INFO("Publishing the filtered PointCloud.");
    pc_pub_.publish(filtered_msg);
  }

  count++; // Increment the counter for message throttling
}
