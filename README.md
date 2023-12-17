# PointCloud Filter ROS Node

This ROS package implements a node that filters PointCloud2 messages based on intensity and provides the ability to throttle messages.

## Overview

The `pointcloud_filter_node_hw` subscribes to a PointCloud2 `/input_pointcloud` topic, applies a filtering process, and publishes the filtered PointCloud2 data to the `/filtered_pointcloud` topic. This node allows the removal of points with intensities below *1000* and includes a throttle feature to control message publishing frequency (set to *2* as default).

## Requirements

- ROSdistro Noetic
- ROSversion 1.15.9
- Dependencies: [roscpp, sensor_msgs, rospcl]
- TurtleBot3 used for simulation and testing

## Installation

1. Clone this repository into your catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone <repository_url>
    ```

2. Build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage

1. Launch the `pointcloud_filter_node_hw` node along with RViz for visualization:

    ```bash
    roslaunch pointcloud_filter pointcloud_filter_node.launch
    ```

2. Set parameters for the node:

    Edit the launch file or use `rosparam` to adjust the throttle rate and intensity threshold as needed.

3. Visualize the filtered PointCloud data:

    The launch file opens RViz and loads the provided `pc_visualization_config.rviz` configuration file to visualize the filtered PointCloud data.

4. Optional. To run the node without RVIZ configuration:
    ```bash
    rosrun pointcloud_filter pc_filter_node
    ```

## Parameters

- `throttle_rate`: Controls the message publishing frequency.
- `intensity_threshold`: Sets the threshold for intensity filtering.

## Nodes

### ROS Node Info
- `rosnode info /pointclound_filter_node_hw`

```
Node [/pointcloud_filter_node_hw]
Publications:
 * /filtered_pointcloud [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /input_pointcloud [sensor_msgs/PointCloud2]

Services:
 * /pointcloud_filter_node_hw/get_loggers
 * /pointcloud_filter_node_hw/set_logger_level
```
#### Subscribed Topics

- `/input_pointcloud` (sensor_msgs/PointCloud2): Input PointCloud data to be filtered.

#### Published Topics

- `/filtered_pointcloud` (sensor_msgs/PointCloud2): Filtered PointCloud data.

## Launch Files

- `pointcloud_filter.launch`: Launches the `pc_filter_node` with specified parameters and RViz visualization.

