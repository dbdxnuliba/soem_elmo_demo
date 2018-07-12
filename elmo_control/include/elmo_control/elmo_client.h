#ifndef ELMO_CLIENT_H
#define ELMO_CLIENT_H

#include <ethercat_manager/ethercat_manager.h>
#include <cstdint>



namespace ethercat {
class EtherCatManager;
} //end of namespace ethercat

namespace  elmo_control {
/**
  * @brief Structure for input (TXPDO)
  *
  */
typedef struct{
  int32_t position;
  int16_t torque;
  int32_t velocity;
  uint16_t status;
  int8_t operation_mode;
  uint16_t current;
}ElmoInput;

/**
  * @brief Structure for Outout (RXPDO)
  *
  */
typedef struct{
  uint16_t controlword;
  uint8_t  operation_mode;
  uint32_t vel;
}ElmoOutput;

/**
  * @brief StatusWord
  * NOT_READY -> SWITCH_DISABLED -> READY_SWITCH -> SWITCHED_ON -> OPERATION_ENABLED
  */
typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;


/**
 * @brief The ElmoClient class
 * This class provides input output access to and from the drive
 */
class ElmoClient{
public:
  /**
   * @brief ElmoClient Constructor sets the number of drives and the manager
   * @param manager the ethercat manager
   * @param slave_no the number of slaces
   */
  ElmoClient(ethercat::EtherCatManager& manager, int slave_no);

  /**
   * @brief writeOutputs writes output to the manager
   * @param output elmoOutput
   */
  void writeOutputs(const ElmoOutput& output);

  /**
   * @brief readInputs reads input from the manager
   * @return elmoInput
   */
  ElmoInput readInputs() const;

  /**
   * @brief readOutputs reads output from the manager
   * @return elmoOutput
   */
  ElmoOutput readOutputs() const;

  /**
   * @brief reset resets the drive
   */
  void reset();

  /**
   * @brief servoOn runs through the state machine and takes drive to operation enabled mode
   */
  void servoOn();

  /**
   * @brief servoOff runs through reverse state machine and takes drive to shutdowm
   */
  void servoOff();

  /**
   * @brief printPDSStatus prints the current PDS status
   * @param input
   */
  void printPDSStatus(const ElmoInput input) const;


private:
  /**
   * @brief getPDSStatus get the current PDS status
   * @param input elmoInput
   * @return
   */
  PDS_STATUS getPDSStatus(const ElmoInput input) const;

  ethercat::EtherCatManager& manager_;
  const int slave_no_;
};


}//end of namespace elmo_control

#endif // ELMO_CLIENT_H
