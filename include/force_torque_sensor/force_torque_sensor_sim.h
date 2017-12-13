
#ifndef FORCETORQUESENSORSIM_INCLUDEDEF_H
#define FORCETORQUESENSORSIM_INCLUDEDEF_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <force_torque_sensor/force_torque_sensor_hw.h>

namespace force_torque_sensor
{

class ForceTorqueSensorSim : public  hardware_interface::ForceTorqueSensorHW
{
public:
  ForceTorqueSensorSim();
  ForceTorqueSensorSim(int type, std::string path, int baudrate, int base_identifier);
  ~ForceTorqueSensorSim() {};

  bool init();
  bool initCommunication(int type, std::string path, int baudrate, int base_identifier);
  bool readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz);
  bool readDiagnosticADCVoltages(int index, short int& value);

  void subscribeData(const geometry_msgs::Twist::ConstPtr& msg);
  geometry_msgs::WrenchStamped joystick_data;

private:
  ros::NodeHandle nh_;
  ros::Subscriber force_input_subscriber;
};

}
#endif

