
#ifndef FORCETORQUESENSORHW_INCLUDEDEF_H
#define FORCETORQUESENSORHW_INCLUDEDEF_H

namespace hardware_interface
{

class ForceTorqueSensorHW
{
public:
  ForceTorqueSensorHW() {};
  ForceTorqueSensorHW(int type, std::string path, int baudrate, int base_identifier) {};
  virtual ~ForceTorqueSensorHW() {};

  virtual bool init() { return true; };
  virtual bool initCommunication(int type, std::string path, int baudrate, int base_identifier) { return true; };
  virtual bool readFTData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz) { return true; };
  virtual bool readDiagnosticADCVoltages(int index, short int& value) { return true; };
};

}
#endif
