# system_model

## Overview

A ROS package for modeling and tracking the state of a system through an internal Unscented Kalman Filter (UKF). The user creates a plugin that defines the system's model (for state prediction), and subscribes to ROS messages for reading sensor measurements (for state updating). The user's plugin can then be loaded and run by the system_model node, which will continuously calculate and publish the current state and covariance of the system via ROS messages.

**Keywords:** model state_estimation kalman_filter plugin

### License

This source code is released under an [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The system_model package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [kalman_filter](https://github.com/pcdangio/ros-kalman_filter) (ROS Kalman Filter package)
- [system_model_msgs](https://github.com/pcdangio/ros-system_model_msgs) (ROS system_model messages)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/pcdangio/ros-system_model.git system_model
    cd ../
    catkin_make

## Usage

First, be sure to set the [~/plugin_path parameter](#parameters) to point to the system_model plugin.

You may then run the system_model node with:

    rosrun system_model node

## Nodes

### node

A node that loads a system model plugin to continuously calculate and publish the current state and covariance.

#### Published Topics
* **`~/state/X`** ([system_model_msgs/variable](https://github.com/pcdangio/ros-system_model_msgs/blob/main/msg/variable.msg))

    Provides the current estimate of a state variable's value and variance. A separate topic is published for each variable in the state.
    The "X" in the above topic name will be set to the name of each variable specified by the system model plugin.

#### Parameters

* **`~/plugin_path`** (string, default: "")

    The path to the system_model plugin that will be run. This should point to the plugin's *.so library file.

* **`~/loop_rate`** (double, default: 100Hz)

    The rate, in Hz, that the system_model's internal UKF will predict/update the state. For maximum performance,
    set the loop rate to meet or beat the fastest sensor rate.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-system_model/issues).

[ROS]: http://www.ros.org