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

#include <force_torque_sensor/force_torque_sensor_handle.h>

#include <pluginlib/class_loader.h>

using namespace force_torque_sensor;

ForceTorqueSensorHandle::ForceTorqueSensorHandle(ros::NodeHandle& nh, hardware_interface::ForceTorqueSensorHW *sensor, std::string sensor_name, std::string output_frame) :
    hardware_interface::ForceTorqueSensorHandle(sensor_name, output_frame, interface_force_, interface_torque_), nh_(nh), calibration_params_{nh.getNamespace()+"/Calibration/Offset"}, CS_params_{nh.getNamespace()}, HWComm_params_{nh.getNamespace()+"/HWComm"}, FTS_params_{nh.getNamespace()+"/FTS"}, pub_params_{nh.getNamespace()+"/Publish"}, node_params_{nh.getNamespace()+"/Node"}, gravity_params_{nh.getNamespace()+"/GravityCompensation/params"}
{
    p_Ftc = sensor;
    prepareNode(output_frame);
}

ForceTorqueSensorHandle::ForceTorqueSensorHandle(ros::NodeHandle& nh, std::string sensor_name, std::string output_frame) :
    hardware_interface::ForceTorqueSensorHandle(sensor_name, output_frame, interface_force_, interface_torque_), nh_(nh), calibration_params_{nh.getNamespace()+"/Calibration/Offset"}, CS_params_{nh.getNamespace()}, HWComm_params_{nh.getNamespace()+"/HWComm"}, FTS_params_{nh.getNamespace()+"/FTS"}, pub_params_{nh.getNamespace()+"/Publish"}, node_params_{nh.getNamespace()+"/Node"}, gravity_params_{nh.getNamespace()+"/GravityCompensation/params"}
{
    node_params_.fromParamServer();

    //load specified sensor HW
    sensor_loader_.reset (new pluginlib::ClassLoader<hardware_interface::ForceTorqueSensorHW>("force_torque_sensor", "hardware_interface::ForceTorqueSensorHW"));
    if (!node_params_.sensor_hw.empty())
      {
        try
          {
            sensor_.reset (sensor_loader_->createUnmanagedInstance (node_params_.sensor_hw));
            ROS_INFO_STREAM ("Sensor type " << node_params_.sensor_hw << " was successfully loaded.");
            
            p_Ftc = sensor_.get();
            prepareNode(output_frame);
          }
        catch (pluginlib::PluginlibException& e)
          {
            ROS_ERROR_STREAM ("Plugin failed to load:" << e.what());
          }
      }
    else
      {
        ROS_ERROR_STREAM ("Failed to getParam 'sensor_hw' (namespace: " << nh.getNamespace() << ").");
        ROS_ERROR ("Sensor hardware failed to load");
      }
}


void ForceTorqueSensorHandle::prepareNode(std::string output_frame)
{    
    ROS_DEBUG_STREAM ("Sensor is using namespace '" << nh_.getNamespace() << "'.");

    transform_frame_ = output_frame;

    reconfigCalibrationSrv_.setCallback(boost::bind(&ForceTorqueSensorHandle::reconfigureCalibrationRequest, this, _1, _2));

    calibration_params_.fromParamServer();
    CS_params_.fromParamServer();
    HWComm_params_.fromParamServer();
    FTS_params_.fromParamServer();
    pub_params_.fromParamServer();
    node_params_.fromParamServer();

    if (calibration_params_.n_measurements <= 0)
    {
        ROS_WARN("Parameter 'Calibration/n_measurements' is %d (<=0) using default: 20", calibration_params_.n_measurements);
        calibration_params_.n_measurements = 20;
    }
    else {
        calibration_params_.n_measurements = std::abs(calibration_params_.n_measurements);
    }

    m_isInitialized = false;
    m_isCalibrated = false;
    srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueSensorHandle::srvCallback_Init, this);
    srvServer_CalculateAverageMasurement_ = nh_.advertiseService("CalculateAverageMasurement", &ForceTorqueSensorHandle::srvCallback_CalculateAverageMasurement, this);
    srvServer_CalculateOffset_ = nh_.advertiseService("CalculateOffsets", &ForceTorqueSensorHandle::srvCallback_CalculateOffset, this);
    srvServer_DetermineCoordianteSystem_ = nh_.advertiseService("DetermineCoordinateSystem", &ForceTorqueSensorHandle::srvCallback_DetermineCoordinateSystem, this);
    srvServer_Temp_ = nh_.advertiseService("GetTemperature", &ForceTorqueSensorHandle::srvReadDiagnosticVoltages, this);
    srvServer_ReCalibrate = nh_.advertiseService("CalculateOffsetsWithoutGravity", &ForceTorqueSensorHandle::srvCallback_CalculateOffsetWithoutGravity, this);
    srvServer_SetSensorOffset = nh_.advertiseService("SetSensorOffset", &ForceTorqueSensorHandle::srvCallback_setSensorOffset, this);
 
    p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);

    ftUpdateTimer_ = nh_.createTimer(ros::Rate(node_params_.ft_pub_freq), &ForceTorqueSensorHandle::updateFTData, this, false, false);
    ftPullTimer_ = nh_.createTimer(ros::Rate(node_params_.ft_pull_freq), &ForceTorqueSensorHandle::pullFTData, this, false, false);

    //Wrench Publisher
    if(pub_params_.sensor_data){
        sensor_data_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "sensor_data", 1);
    }

    if(pub_params_.transformed_data){
        transformed_data_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "transformed_data", 1);
    }

    if(pub_params_.output_data){
        output_data_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "output_data", 1);
    }

    ros::NodeHandle filters_nh("~");
    ROS_DEBUG_STREAM("Filters are using namespace '" << filters_nh.getNamespace() << "'.");

    //Median Filter
    if(filters_nh.hasParam("MovingMeanFilter")) {
        ROS_DEBUG("Using MovingMeanFilter");
        useMovingMean = true;
        moving_mean_filter_->configure(filters_nh.getNamespace()+"/MovingMeanFilter");

        if(pub_params_.moving_mean){
            moving_mean_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "moving_mean", 1);
        }
    } else {
        ROS_WARN("MovingMeanFilter configuration not found. It will not be used!");
    }

    //Low Pass Filter
    if(filters_nh.hasParam("LowPassFilter")) {
        ROS_DEBUG("Using LowPassFilter");
        useLowPassFilter = true;
        low_pass_filter_->configure(filters_nh.getNamespace()+"/LowPassFilter");

        if(pub_params_.low_pass){
            low_pass_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "low_pass", 1);
        }
    } else {
        ROS_WARN("LowPassFilter configuration not found. It will not be used!");
    }

    //Gravity Compenstation
    if(filters_nh.hasParam("GravityCompensation")) {
        ROS_DEBUG("Using GravityCompensation");
        useGravityCompensator = true;
        gravity_compensator_->configure(filters_nh.getNamespace()+"/GravityCompensation");

        // Read GravityParams also here to use for recalibration (see: srvCallback_CalculateOffsetWithoutGravity Function)
        gravity_params_.fromParamServer();
        if(pub_params_.gravity_compensated){
            gravity_compensated_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "gravity_compensated", 1);
        }
    } else {
        ROS_WARN("GravityCompensation configuration not found. It will not be used!");
    }

    //Threshold Filter
    if(filters_nh.hasParam("ThresholdFilter")) {
        ROS_DEBUG("Using ThresholdFilter");
        useThresholdFilter = true;
        threshold_filter_->configure(filters_nh.getNamespace()+"/ThresholdFilter");

        if(pub_params_.threshold_filtered){
            threshold_filtered_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "threshold_filtered", 1);
        }
    } else {
        ROS_WARN("ThresholdFilter configuration not found. It will not be used!");
    }

    p_Ftc->initCommunication(HWComm_params_.type, HWComm_params_.path, HWComm_params_.baudrate, FTS_params_.base_identifier);

    if (FTS_params_.auto_init)
    {
        std::string msg;
        bool success;
        ROS_DEBUG("Starting Autoinit...");
        init_sensor(msg, success);
        ROS_INFO("Autoinit: %s", msg.c_str());
    }
}

void ForceTorqueSensorHandle::init_sensor(std::string& msg, bool& success)
{
    if (!m_isInitialized)
    {
        // read return init status and check it!
        if (p_Ftc->init())
        {
            // start timer for reading FT-data
            ftPullTimer_.start();
            ros::Duration(0.1).sleep(); //wait for measurements to come

            m_isInitialized = true;
            success = true;
            msg = "FTS initialized!";

            // Calibrate sensor
            if (calibration_params_.isStatic)
            {
                std::map<std::string,double> forceVal,torqueVal;
                forceVal = calibration_params_.force;
                torqueVal = calibration_params_.torque;

                ROS_INFO("Using static Calibration Offset from paramter server with parametes Force: x:%f, y:%f, z:%f; Torque: x: %f, y:%f, z:%f;",
                    forceVal["x"], forceVal["y"], forceVal["z"],
                    torqueVal["x"], torqueVal["y"], torqueVal["z"]
                );

                offset_.force.x = forceVal["x"];
                offset_.force.y = forceVal["y"];
                offset_.force.z = forceVal["z"];
                offset_.torque.x = torqueVal["x"];
                offset_.torque.y = torqueVal["y"];
                offset_.torque.z = torqueVal["z"];

                apply_offset = true;
                m_isCalibrated = true;
            }
            else
            {
                ROS_INFO("Calibrating sensor. Plase wait...");
                geometry_msgs::Wrench temp_offset;
                if (not calculate_offset(true, &temp_offset))
                {
                    success = false;
                    msg = "Calibration failed! :/";
                }
            }
        }
        else
        {
            m_isInitialized = false;
            success = false;
            msg = "FTS could not be initialized! :/";
            ROS_FATAL("FTS Hardware could not be initialized");
        }
        ftUpdateTimer_.start();
    }
}

bool ForceTorqueSensorHandle::srvCallback_Init(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    std::string msg;
    bool success;

    init_sensor(msg, success);
    res.message = msg;
    res.success = success;

    return true;
}

bool ForceTorqueSensorHandle::srvCallback_CalculateAverageMasurement(force_torque_sensor::CalculateAverageMasurement::Request& req, force_torque_sensor::CalculateAverageMasurement::Response& res)
{
    if (m_isInitialized)
    {
        res.success = true;
        res.message = "Measurement successfull! :)";
        res.measurement = makeAverageMeasurement(req.N_measurements, req.T_between_meas, req.frame_id);
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised! :/";
    }

    return true;
}

bool ForceTorqueSensorHandle::srvCallback_CalculateOffset(force_torque_sensor::CalculateSensorOffset::Request& req, force_torque_sensor::CalculateSensorOffset::Response& res)
{
    if (m_isInitialized)
    {
        if (calculate_offset(req.apply_after_calculation, &res.offset))
        {
            res.success = true;
            res.message = "Calibration successfull! :)";
        }
        else
        {
            res.success = false;
            res.message = "Calibration failed! :/";
        }
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised! :/";
    }

    return true;
}

bool ForceTorqueSensorHandle::srvCallback_CalculateOffsetWithoutGravity(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    if (!m_isInitialized)
    {
        ROS_WARN("FTS-Node is not initialized, please initialize first!");
        res.success = false;
        res.message = "Failed to recalibrate because Node is not initiliazed.";
        return true;
    }
    if (!(nh_.hasParam("force") && nh_.hasParam("CoG_x") && nh_.hasParam("CoG_y") && nh_.hasParam("CoG_z")))
    {
        ROS_ERROR("Cannot use dynamic recalibration without all values for Gravity Compensation, set parameters or use "
                  "'Calibrate' service instead.");
        res.success = false;
        res.message = "Failed to recalibrate because of missing Parameters for Gravity Compensation.";
        return true;
    }
    geometry_msgs::Vector3Stamped gravity, gravity_transformed;
    geometry_msgs::Vector3 cog;
    double force_value;
    cog.x = gravity_params_.CoG_x;
    cog.y = gravity_params_.CoG_y;
    cog.z = gravity_params_.CoG_z;
    force_value = gravity_params_.force;
    gravity.vector.z = -force_value;
    tf2::doTransform(gravity, gravity_transformed, p_tfBuffer->lookupTransform(node_params_.sensor_frame, transform_frame_, ros::Time(0)));
    geometry_msgs::Wrench offset;
    calculate_offset(false, &offset);
    offset_.force.x -= gravity_transformed.vector.x;
    offset_.force.y -= gravity_transformed.vector.y;
    offset_.force.z -= gravity_transformed.vector.z;
    offset_.torque.x -= (gravity_transformed.vector.y * cog.z - gravity_transformed.vector.z * cog.y);
    offset_.torque.y -= (gravity_transformed.vector.z * cog.x - gravity_transformed.vector.x * cog.z);
    offset_.torque.z -= (gravity_transformed.vector.x * cog.y - gravity_transformed.vector.y * cog.x);
    res.success = true;
    res.message = "Successfully recalibrated FTS!";
    return true;
}

bool ForceTorqueSensorHandle::srvCallback_setSensorOffset(force_torque_sensor::SetSensorOffset::Request &req, force_torque_sensor::SetSensorOffset::Response &res)
{
    offset_.force.x = req.offset.force.x;
    offset_.force.y = req.offset.force.y;
    offset_.force.z = req.offset.force.z;
    offset_.torque.x = req.offset.torque.x;
    offset_.torque.y = req.offset.torque.y;
    offset_.torque.z = req.offset.torque.z;

    res.success = true;
    res.message = "Offset is successfully set!";
    return true;
}


bool ForceTorqueSensorHandle::calculate_offset(bool apply_after_calculation, geometry_msgs::Wrench *new_offset)
{
    apply_offset = false;
    ongoing_offset_calculation = true;
    ROS_DEBUG("Calibrating using %d measurements and %f s pause between measurements.", calibration_params_.n_measurements, calibration_params_.T_between_meas);
    geometry_msgs::Wrench temp_offset = makeAverageMeasurement(calibration_params_.n_measurements, calibration_params_.T_between_meas);

    ongoing_offset_calculation = false;
    if (apply_after_calculation) {
        offset_ = temp_offset;
    }
    apply_offset = true;

    ROS_DEBUG("Calculated Calibration Offset: Fx: %f; Fy: %f; Fz: %f; Mx: %f; My: %f; Mz: %f", temp_offset.force.x, temp_offset.force.y, temp_offset.force.z, temp_offset.torque.x, temp_offset.torque.y, temp_offset.torque.z);

    m_isCalibrated = true;
    *new_offset = temp_offset;

    return m_isCalibrated;
}

geometry_msgs::Wrench ForceTorqueSensorHandle::makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id)
{
    geometry_msgs::Wrench measurement;
    int num_of_errors = 0;
    ros::Duration duration(time_between_meas);
    for (int i = 0; i < number_of_measurements; i++) {
      geometry_msgs::Wrench output;
      if (not frame_id.empty()) {
        if (not transform_wrench(frame_id, node_params_.sensor_frame, moving_mean_filtered_data.wrench, output)) {
            num_of_errors++;
            if (num_of_errors > 200) {
                return measurement;
            }
            i--;
            continue;
        }
      }
      else {
        output = moving_mean_filtered_data.wrench;
      }

      measurement.force.x += output.force.x;
      measurement.force.y += output.force.y;
      measurement.force.z += output.force.z;
      measurement.torque.x += output.torque.x;
      measurement.torque.y += output.torque.y;
      measurement.torque.z+= output.torque.z;

      duration.sleep();
    }
    measurement.force.x /= number_of_measurements;
    measurement.force.y /= number_of_measurements;
    measurement.force.z /= number_of_measurements;
    measurement.torque.x /= number_of_measurements;
    measurement.torque.y /= number_of_measurements;
    measurement.torque.z /= number_of_measurements;
    return measurement;
}


// TODO: make this to use filtered data (see calibrate)
bool ForceTorqueSensorHandle::srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request& req,
                                                              std_srvs::Trigger::Response& res)
{
    if (m_isInitialized && m_isCalibrated)
    {
        double angle;

        ROS_INFO("Please push FTS with force larger than 10 N in desired direction of new axis %d",
                 CS_params_.push_direction);

        for (int i = 0; i < CS_params_.n_measurements; i++)
        {
            int status = 0;
            double Fx, Fy, Fz, Tx, Ty, Tz = 0;
            p_Ftc->readFTData(status, Fx, Fy, Fz, Tx, Ty, Tz);

            angle += atan2(Fy, Fx);

            usleep(CS_params_.T_between_meas);
        }

        angle /= CS_params_.n_measurements;

        if (CS_params_.push_direction)
        {
            angle -= M_PI / 2;
        }

        ROS_INFO("Please rotate your coordinate system for %f rad (%f deg) around z-axis", angle, angle / M_PI * 180.0);

        res.success = true;
        res.message = "CoordianteSystem  successfull! :)";
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised or not calibrated! :/";
    }

    return true;
}

bool ForceTorqueSensorHandle::srvReadDiagnosticVoltages(force_torque_sensor::DiagnosticVoltages::Request& req,
                                                  force_torque_sensor::DiagnosticVoltages::Response& res)
{
    p_Ftc->readDiagnosticADCVoltages(req.index, res.adc_value);

    return true;
}

void ForceTorqueSensorHandle::pullFTData(const ros::TimerEvent &event)
{
//     ros::Time timestamp = ros::Time::now();  
    if (p_Ftc->readFTData(0, sensor_data.wrench.force.x, sensor_data.wrench.force.y, sensor_data.wrench.force.z,
                                                        sensor_data.wrench.torque.x, sensor_data.wrench.torque.y, sensor_data.wrench.torque.z)
        != false)
    {
        sensor_data.header.stamp = ros::Time::now();
        sensor_data.header.frame_id = node_params_.sensor_frame;
        if (apply_offset) {
            sensor_data.wrench.force.x  -= offset_.force.x;
            sensor_data.wrench.force.y  -= offset_.force.y;
            sensor_data.wrench.force.z  -= offset_.force.z;
            sensor_data.wrench.torque.x -= offset_.torque.x;
            sensor_data.wrench.torque.y -= offset_.torque.y;
            sensor_data.wrench.torque.z -= offset_.torque.z;
        }

        //lowpass
        low_pass_filtered_data.header = sensor_data.header;
        if(useLowPassFilter and !ongoing_offset_calculation){
            low_pass_filter_->update(sensor_data,low_pass_filtered_data);
        }
        else low_pass_filtered_data = sensor_data;

        //moving_mean
        moving_mean_filtered_data.header = low_pass_filtered_data.header;
        if(useMovingMean){
            moving_mean_filter_->update(low_pass_filtered_data, moving_mean_filtered_data);
        }
        else moving_mean_filtered_data = low_pass_filtered_data;


        if (ft_data_lock_.try_lock_for(std::chrono::milliseconds(1))) {
            prefiltered_data_ = moving_mean_filtered_data;
            ft_data_lock_.unlock();
        }
        else {
            ROS_WARN("pullFTData: Waiting for Lock for 1ms not successful! If this happens more often You might have a problem with timing of measurements!");
        }


        if(pub_params_.sensor_data)
            if (sensor_data_pub_->trylock()){
                sensor_data_pub_->msg_ = sensor_data;
                sensor_data_pub_->unlockAndPublish();
            }

        if(pub_params_.low_pass)
            if (low_pass_pub_->trylock()){
                low_pass_pub_->msg_ = low_pass_filtered_data;
                low_pass_pub_->unlockAndPublish();
            }

        if(pub_params_.moving_mean)
            if (moving_mean_pub_->trylock()){
                moving_mean_pub_->msg_ = moving_mean_filtered_data;
                moving_mean_pub_->unlockAndPublish();
            }
    }
    
//      std::cout << (ros::Time::now() - timestamp).toNSec()/1000.0 << " ms" << std::endl;
}

void ForceTorqueSensorHandle::filterFTData()
{
    if (ft_data_lock_.try_lock_for(std::chrono::milliseconds(1))) {
        filtered_data_input_ = prefiltered_data_;
        ft_data_lock_.unlock();
    }
    else {
        ROS_WARN("filterFTData: Waiting for Lock for 1ms not successful! Using old data! If this happens more often You might have a problem with timing of measurements!");
    }

    transformed_data.header.stamp = filtered_data_input_.header.stamp;
    transformed_data.header.frame_id = transform_frame_;
    if (transform_wrench(transform_frame_, node_params_.sensor_frame, filtered_data_input_.wrench, transformed_data.wrench))
    {
      //gravity compensation
      if(useGravityCompensator)
      {
          gravity_compensator_->update(transformed_data, gravity_compensated_force);
      }
      else gravity_compensated_force = transformed_data;

      //treshhold filtering
      if(useThresholdFilter)
      {
          threshold_filter_->update(gravity_compensated_force, threshold_filtered_force);
      }
      else threshold_filtered_force = gravity_compensated_force;

      if(pub_params_.transformed_data)
         if (transformed_data_pub_->trylock()){
              transformed_data_pub_->msg_ = transformed_data;
              transformed_data_pub_->unlockAndPublish();
         }
      if(pub_params_.gravity_compensated && useGravityCompensator)
         if (gravity_compensated_pub_->trylock()){
              gravity_compensated_pub_->msg_ = gravity_compensated_force;
              gravity_compensated_pub_->unlockAndPublish();
         }

      if(pub_params_.threshold_filtered && useThresholdFilter)
         if (threshold_filtered_pub_->trylock()){
             threshold_filtered_pub_->msg_ = threshold_filtered_force;
             threshold_filtered_pub_->unlockAndPublish();
        }
      output_data = threshold_filtered_force;
    }
    else {
      output_data = moving_mean_filtered_data;
    }
}

bool ForceTorqueSensorHandle::transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench& transformed)
{
    if (output_transform_.transform.rotation.x == 0 and output_transform_.transform.rotation.y  == 0 and output_transform_.transform.rotation.z  == 0 and output_transform_.transform.rotation.w == 0) {
        ROS_INFO_THROTTLE(1, "FTS Transform not yet initalized, Trying to get one...");
        if (!updateTransform(transform_frame_, node_params_.sensor_frame)) {
            ROS_WARN_STREAM_THROTTLE(1, "Transform between '" << node_params_.sensor_frame << "' and '" << transform_frame_ << "' not available! Probably not ready yet. If get this mesage more often, please check URDF descritption of Your robot!");
            return false;
        }
        else if (node_params_.static_application) {
            ROS_INFO("Got FTS Transform for static application!");
        }
    }
    if (!node_params_.static_application) {
        if (!updateTransform(goal_frame, source_frame)) {
            return false;
        }
    }

    tf2::doTransform(wrench, transformed, output_transform_);
    return true;
}

bool ForceTorqueSensorHandle::updateTransform(std::string goal_frame, std::string source_frame)
{
    try
    {
        output_transform_ = p_tfBuffer->lookupTransform(goal_frame, source_frame, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR_THROTTLE(2, "%s", ex.what());
        return false;
    }
    return true;
}

void ForceTorqueSensorHandle::reconfigureCalibrationRequest(force_torque_sensor::CalibrationConfig& config, uint32_t level)
{
    calibration_params_.fromConfig(config);
}

void ForceTorqueSensorHandle::updateFTData(const ros::TimerEvent& event)
{
    filterFTData();

    if(pub_params_.output_data) {
        if (output_data_pub_->trylock()){
            output_data_pub_->msg_ = output_data;
            output_data_pub_->unlockAndPublish();
        }
    }

    interface_force_[0] = output_data.wrench.force.x;
    interface_force_[1] = output_data.wrench.force.y;
    interface_force_[2] = output_data.wrench.force.z;

    interface_torque_[0] = output_data.wrench.torque.x;
    interface_torque_[1] = output_data.wrench.torque.y;
    interface_torque_[2] = output_data.wrench.torque.z;
}
