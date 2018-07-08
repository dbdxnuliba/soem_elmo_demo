#ifndef ELMO_CLIENT_H
#define ELMO_CLIENT_H

#include <ethercat_manager/ethercat_manager.h>
#include <cstdint>



namespace ethercat {
class EtherCatManager;
}

namespace  elmo_control {


typedef struct{
  int32_t position;
  int16_t torque;
  int32_t velocity;
  uint16_t status;
  int8_t operation_mode;
  uint16_t current;
}ElmoInput;

typedef struct{
  uint16_t controlword;
  uint8_t  operation_mode;
  uint32_t vel;
}ElmoOutput;


typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;

typedef enum {NO_MODE_CHANGE, PROFILE_POSITION_MODE, VELOCITY_MODE, PROFILE_VELOCITY_MODE, TORQUE_PROFILE_MODE, HOMING_MODE, INTERPOLATED_POSITION_MODE, CYCLIC_SYNCHRONOUS_POSITION_MODE, CYCLIC_SYNCHRONOUS_VELOCITY_MODE, CYCLIC_SYNCHRONOUS_TORQUE_MODE} PDS_OPERATION;

typedef enum{}PDS_CONTROL;

class ElmoClient{
public:
  ElmoClient(ethercat::EtherCatManager& manager, int slave_no);
  void writeOutputs(const ElmoOutput& output);
  ElmoInput readInputs() const;
  ElmoOutput readOutputs() const;
  void reset();
  void servoOn();
  void servoOff();
  void setTorqueForEmergencyStop(double val);
  void setOverLoadLevel(double val);
  void setOverSpeedLevel(double val);
  void setMotorWorkingRange(double val);
  void setProfileVeclocity(uint32_t val);
  void setInterpolationTimePeriod(int us);
  void printPDSStatus(const ElmoInput input) const;
  void printPDSOperation(const ElmoInput input) const;
  void printPDSControl(const ElmoInput input) const;

private:
  PDS_STATUS getPDSStatus(const ElmoInput input) const;
  PDS_OPERATION getPDSOperation(const ElmoInput input) const;
  PDS_CONTROL getPDSControl(const ElmoInput input) const;
  ethercat::EtherCatManager& manager_;
  const int slave_no_;
};


}
#endif // ELMO_CLIENT_H
