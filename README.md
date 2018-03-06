# force_torque_sensor

ROS implementation of generic force-torque sensor. The implementation implements pluginlib infrastructure and node for integration and use in ROS.

The package provides a simulation of a FTS where a joystic is used for data input. This is often usefull for tests.

Note: Current version of the package depends on iirob/iirob_filters package for filtering capabilities. To install this package clone the [git repository](https://github.com/iirob/iirob_filters) into src folder of your workspace 

## ROS Distro Support

|         | Kinetic |
|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://github.com/KITrobotics/force_torque_sensor/tree/kinetic-devel) |
| Status  | [![Build Status](https://travis-ci.org/KITrobotics/force_torque_sensor.svg?branch=kinetic-devel)](https://travis-ci.org/KITrobotics/force_torque_sensor) |
| Version | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=force_torque_sensor) |
