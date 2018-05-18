#ifndef ELMO_CLIENT_H
#define ELMO_CLIENT_H

#include <ethercat_manager/ethercat_manager.h>
#include <soem/osal.h>

namespace ethercat {
class EtherCatManager;
}

namespace  elmo_control {

typedef struct{
  uint16 statusword;
  uint8 operation_mode;
  uint16 drivetemp;
  uint16 current_actual_value;
  uint32 position_actual_value;
  uint32 velocity_actual_value;
  uint16 analog_input_1;
  uint16 dc_supply_5v;
  uint32 dc_link_circuit_voltage;
  uint32 digital_input;
}ElmoInput;

typedef struct{
  uint16 controlword;
  uint8 operation_mode;
  uint32 target_velocity;
}ELmoOutput;

typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;

typedef enum {NO_MODE_CHANGE, PROFILE_POSITION_MODE, VELOCITY_MODE, PROFILE_VELOCITY_MODE, TORQUE_PROFILE_MODE, HOMING_MODE, INTERPOLATED_POSITION_MODE, CYCLIC_SYNCHRONOUS_POSITION_MODE, CYCLIC_SYNCHRONOUS_VELOCITY_MODE, CYCLIC_SYNCHRONOUS_TORQUE_MODE} PDS_OPERATION;

typedef enum{}PDS_CONTROL;

class ElmoClient{
public:
  ElmoClient(ethercat::EtherCatManager& manager, int slave_no);
  void writeOutputs(const ELmoOutput& output);
  ElmoInput readInputs() const;
  ELmoOutput readOutputs() const;
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
