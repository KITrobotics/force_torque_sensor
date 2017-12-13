#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <force_torque_sensor/force_torque_sensor_sim.h>
#include <force_torque_sensor/NodeConfigurationParameters.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_torque_sensor_node");

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::NodeHandle nh("/fts");

    force_torque_sensor::NodeConfigurationParameters node_params_(nh.getNamespace()+"/Node");

    node_params_.fromParamServer();

    new force_torque_sensor::ForceTorqueSensorHandle(nh, node_params_.sensor_frame,node_params_.transform_frame);

    ROS_INFO("ForceTorque Sensor Node running.");

    ros::waitForShutdown();

    return 0;
}
