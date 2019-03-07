#include <force_torque_sensor/force_torque_sensor_sim.h>

#include <pluginlib/class_list_macros.h>

using namespace force_torque_sensor;

ForceTorqueSensorSim::ForceTorqueSensorSim()
{
  std::cout<<"ForceTorqueSensorSim"<<std::endl;
}

ForceTorqueSensorSim::ForceTorqueSensorSim(int type, std::string path, int baudrate, int base_identifier)
{
  std::cout<<"ForceTorqueSensorSim"<<std::endl;
}

bool ForceTorqueSensorSim::init() {
    ros::NodeHandle nh_("~");
    force_input_subscriber = nh_.subscribe("/cmd_force", 1, &ForceTorqueSensorSim::subscribeData, this);
}

bool ForceTorqueSensorSim::initCommunication(int type, std::string path, int baudrate, int base_identifier) {
    std::cout<<"ForceTorqueSensorSim"<<std::endl;
}

void ForceTorqueSensorSim::subscribeData(const geometry_msgs::Twist::ConstPtr& msg){
    joystick_data.wrench.force.x = msg->linear.x;
    joystick_data.wrench.force.y = msg->linear.y;
    joystick_data.wrench.force.z = msg->linear.z;
    joystick_data.wrench.torque.x = msg->angular.x;
    joystick_data.wrench.torque.y = msg->angular.y;
    joystick_data.wrench.torque.z = msg->angular.z;
}

bool ForceTorqueSensorSim::readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz) {

    Fx = joystick_data.wrench.force.x;
    Fy = joystick_data.wrench.force.y;
    Fz = joystick_data.wrench.force.z;
    Tx = joystick_data.wrench.torque.x;
    Ty = joystick_data.wrench.torque.y;
    Tz = joystick_data.wrench.torque.z;

    return true;
}

bool ForceTorqueSensorSim::readDiagnosticADCVoltages(int index, short int& value){
    std::cout<<"ForceTorqueSensorSim"<<std::endl;
}

bool ForceTorqueSensorSim::init(ros::NodeHandle& root_nh, ros::NodeHandle &sensor_hw_nh) {
    ros::NodeHandle nh_(sensor_hw_nh);
    force_input_subscriber = nh_.subscribe("/cmd_force", 1, &ForceTorqueSensorSim::subscribeData, this);
    hardware_interface::ForceTorqueSensorHandle fts_handle("ForceTorqueSensorSim", "fts_base_link", force_, torque_);
    fts_interface_.registerHandle(fts_handle);
    registerInterface(&fts_interface_);
    return true;
}

void ForceTorqueSensorSim::read(const ros::Time& time, const ros::Duration& period) {
    force_[0] = joystick_data.wrench.force.x;
    force_[1] = joystick_data.wrench.force.y;
    force_[2] = joystick_data.wrench.force.z;
    torque_[0] = joystick_data.wrench.torque.x;
    torque_[1] = joystick_data.wrench.torque.y;
    torque_[2] = joystick_data.wrench.torque.z;
}

PLUGINLIB_EXPORT_CLASS(force_torque_sensor::ForceTorqueSensorSim, hardware_interface::ForceTorqueSensorHW)
PLUGINLIB_EXPORT_CLASS(force_torque_sensor::ForceTorqueSensorSim, hardware_interface::SensorHW)
