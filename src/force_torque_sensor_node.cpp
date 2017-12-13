#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <force_torque_sensor/force_torque_sensor_sim.h>
#include <force_torque_sensor/NodeConfigurationParameters.h>

#include <pluginlib/class_loader.h>

// class ForceTorqueSensorNode
// {
// public:
//     ForceTorqueSensorNode(ros::NodeHandle &nh):node_params_{nh.getNamespace()+"/Node"}
// {
//     node_params_.fromParamServer();
//     bool sim = node_params_.sim;
//     ForceTorqueSensorHW *sensor;
//     if(sim) {
//       sensor = new ForceTorqueSensorSim();
//     }
//     else{
//         sensor = new FtcCtrl();
//     }
//     new ForceTorqueSensorHandle(nh, sensor, node_params_.sensor_frame,node_params_.transform_frame);
// }
// private:
//     force_torque_sensor::NodeConfigurationParameters node_params_;
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_torque_sensor_node");

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::NodeHandle nh("/fts");

//////////////////////////////////

    force_torque_sensor::NodeConfigurationParameters node_params_(nh.getNamespace()+"/Node");

    node_params_.fromParamServer();
    std::string sensor_type_;
    boost::shared_ptr<pluginlib::ClassLoader<hardware_interface::ForceTorqueSensorHW>> sensor_loader_;
    boost::shared_ptr<hardware_interface::ForceTorqueSensorHW> sensor;

    //load specified sensor HW
    sensor_loader_.reset (new pluginlib::ClassLoader<hardware_interface::ForceTorqueSensorHW>("force_torque_sensor", "hardware_interface::ForceTorqueSensorHW"));
    std::string param_name = "/Node/sensor_hw";
    if (nh.getParam (param_name, sensor_type_))
      {
        try
          {
            sensor.reset (sensor_loader_->createUnmanagedInstance (sensor_type_));

            ROS_WARN_STREAM ("IK type " << sensor_type_ << " was successfully loaded.");
          }
        catch (pluginlib::PluginlibException& e)
          {
            ROS_ERROR_STREAM ("Plugin failed to load:" << e.what());
            return false;
          }
      }
    else
      {
        ROS_ERROR_STREAM ("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        ROS_ERROR ("Sensor hardware failed to load");
        return false;
      }

//     ForceTorqueSensorNode ftn(nh);
    new force_torque_sensor::ForceTorqueSensorHandle(nh, sensor.get(), node_params_.sensor_frame,node_params_.transform_frame);

    /////////////////////////////////////////////////////

    ROS_INFO("ForceTorque Sensor Node running.");

    ros::waitForShutdown();

    return 0;
}
