/****************************************************************
 *
 * Copyright 2020 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainer: Denis Štogl, email: denis.stogl@kit.edu
 *
 * Date of update: 2014-2020
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef FORCETORQUESENSORHANDLE_INCLUDEDEF_H
#define FORCETORQUESENSORHANDLE_INCLUDEDEF_H

#include <hardware_interface/force_torque_sensor_interface.h>
#include <force_torque_sensor/force_torque_sensor_hw.h>

#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Trigger.h>
#include <force_torque_sensor/CalculateAverageMasurement.h>
#include <force_torque_sensor/CalculateSensorOffset.h>
#include <force_torque_sensor/DiagnosticVoltages.h>
#include <force_torque_sensor/SetSensorOffset.h>


#include <iirob_filters/gravity_compensation.h>
#include <iirob_filters/GravityCompensationParameters.h>
#include <iirob_filters/low_pass_filter.h>
#include <iirob_filters/threshold_filter.h>
#include <iirob_filters/moving_mean_filter.h>

#include <math.h>
#include <iostream>
#include <mutex>
#include <chrono>

#include <dynamic_reconfigure/server.h>
#include <force_torque_sensor/CoordinateSystemCalibrationParameters.h>
#include <force_torque_sensor/HWCommunicationConfigurationParameters.h>
#include <force_torque_sensor/FTSConfigurationParameters.h>
#include <force_torque_sensor/PublishConfigurationParameters.h>
#include <force_torque_sensor/NodeConfigurationParameters.h>
#include <force_torque_sensor/CalibrationParameters.h>
#include <force_torque_sensor/CalibrationConfig.h>

#include <filters/filter_chain.h>
#include <filters/filter_base.h>
#include <filters/mean.h>
#include <realtime_tools/realtime_publisher.h>

#define PI 3.14159265

namespace force_torque_sensor
{

class ForceTorqueSensorHandle : public hardware_interface::ForceTorqueSensorHandle
{
public:

  ForceTorqueSensorHandle(ros::NodeHandle &nh, hardware_interface::ForceTorqueSensorHW *sensor, std::string sensor_name, std::string output_frame);
  ForceTorqueSensorHandle(ros::NodeHandle &nh, std::string sensor_name, std::string output_frame);
  
  void prepareNode(std::string output_frame);

  void init_sensor(std::string &msg, bool &success);
  bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvCallback_CalculateOffset(force_torque_sensor::CalculateSensorOffset::Request &req, force_torque_sensor::CalculateSensorOffset::Response &res);
  bool srvCallback_CalculateAverageMasurement(force_torque_sensor::CalculateAverageMasurement::Request &req, force_torque_sensor::CalculateAverageMasurement::Response &res);
  bool calculate_offset(bool apply_after_calculation,  geometry_msgs::Wrench *new_offset);
  bool srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvReadDiagnosticVoltages(force_torque_sensor::DiagnosticVoltages::Request &req,
                                 force_torque_sensor::DiagnosticVoltages::Response &res);
  bool srvCallback_CalculateOffsetWithoutGravity(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool srvCallback_setSensorOffset(force_torque_sensor::SetSensorOffset::Request &req,
                                 force_torque_sensor::SetSensorOffset::Response &res);

private:
  void updateFTData(const ros::TimerEvent &event);
  geometry_msgs::Wrench makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id="");

  bool transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench& transformed);

  // Arrays for hardware_interface
  std::string interface_frame_id_;
  double interface_force_[3];
  double interface_torque_[3];

  force_torque_sensor::CoordinateSystemCalibrationParameters CS_params_;
  force_torque_sensor::HWCommunicationConfigurationParameters HWComm_params_;
  force_torque_sensor::FTSConfigurationParameters FTS_params_;
  force_torque_sensor::PublishConfigurationParameters pub_params_;
  force_torque_sensor::NodeConfigurationParameters node_params_;
  force_torque_sensor::CalibrationParameters calibration_params_;
  iirob_filters::GravityCompensationParameters gravity_params_;

  std::string transform_frame_;
  std::string sensor_frame_;

  void pullFTData(const ros::TimerEvent &event);
  geometry_msgs::WrenchStamped filterFTData();

  // Wrenches for dumping FT-Data
  geometry_msgs::WrenchStamped prefiltered_data_, filtered_data_input_; //Communication between read and filter/publish thread
  geometry_msgs::WrenchStamped sensor_data, low_pass_filtered_data, moving_mean_filtered_data, transformed_data, gravity_compensated_force, threshold_filtered_force, output_data; //global variables to avoid on-runtime allocation

  double force_buffer_[3];
  double torque_buffer_[3];
  double force_buffer_transformed_[3];
  double torque_buffer_transformed_[3];

  ros::NodeHandle nh_;

  //FT Data
  hardware_interface::ForceTorqueSensorHW *p_Ftc;
  geometry_msgs::Wrench offset_;
  geometry_msgs::TransformStamped transform_ee_base_stamped;
  tf2_ros::Buffer *p_tfBuffer;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>  *gravity_compensated_pub_, *threshold_filtered_pub_, *transformed_data_pub_, *sensor_data_pub_, *output_data_pub_, *low_pass_pub_, *moving_mean_pub_;

  bool is_pub_gravity_compensated_ = false;
  bool is_pub_threshold_filtered_ = false;
  bool is_pub_transformed_data_ = false;
  bool is_pub_sensor_data_ = false;
  bool is_pub_output_data_ = false;
  bool is_pub_low_pass_ = false;
  bool is_pub_moving_mean_ = false;

  // HWComm parameters
  int HWCommType; // Only important if can is used
  std::string HWCommPath;
  int HWCommBaudrate;
  int ftsBaseID;
  double nodePubFreq, nodePullFreq;
  uint calibrationNMeasurements;
  double calibrationTBetween;
  int coordinateSystemNMeasurements;
  int coordinateSystemTBetween;
  int coordinateSystemPushDirection;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_CalculateAverageMasurement_;
  ros::ServiceServer srvServer_CalculateOffset_;
  ros::ServiceServer srvServer_DetermineCoordianteSystem_;
  ros::ServiceServer srvServer_Temp_;
  ros::ServiceServer srvServer_ReCalibrate;
  ros::ServiceServer srvServer_SetSensorOffset;

  ros::Timer ftUpdateTimer_, ftPullTimer_;

  tf2_ros::TransformListener *p_tfListener;
  tf2::Transform transform_ee_base;

  bool m_isInitialized;
  bool m_isCalibrated;
  bool apply_offset, ongoing_offset_calculation;

  // Variables for Static offset
  bool m_staticCalibration;

  filters::FilterBase<geometry_msgs::WrenchStamped> *moving_mean_filter_ = new iirob_filters::MovingMeanFilter<geometry_msgs::WrenchStamped>();
  filters::FilterBase<geometry_msgs::WrenchStamped> *low_pass_filter_ = new iirob_filters::LowPassFilter<geometry_msgs::WrenchStamped>();
  filters::FilterBase<geometry_msgs::WrenchStamped> *threshold_filter_ = new iirob_filters::ThresholdFilter<geometry_msgs::WrenchStamped>();
  filters::FilterBase<geometry_msgs::WrenchStamped> *gravity_compensator_ = new iirob_filters::GravityCompensator<geometry_msgs::WrenchStamped>();

  bool useGravityCompensator=false;
  bool useThresholdFilter=false;

  bool useMovingMean = false;
  bool useLowPassFilter = false;


  dynamic_reconfigure::Server<force_torque_sensor::CalibrationConfig> reconfigCalibrationSrv_; // Dynamic reconfiguration service

  void reconfigureCalibrationRequest(force_torque_sensor::CalibrationConfig& config, uint32_t level);

  boost::shared_ptr<pluginlib::ClassLoader<hardware_interface::ForceTorqueSensorHW>> sensor_loader_;
  boost::shared_ptr<hardware_interface::ForceTorqueSensorHW> sensor_;

  std::timed_mutex ft_data_lock_;
};

}
#endif
