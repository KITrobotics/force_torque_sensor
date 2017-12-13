
#include <pluginlib/class_list_macros.h>

#include <force_torque_sensor/force_torque_sensor_sim.h>

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


PLUGINLIB_EXPORT_CLASS(ForceTorqueSensorSim, ForceTorqueSensorHW)
