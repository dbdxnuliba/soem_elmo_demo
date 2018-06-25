#include <elmo_control/elmo_client.h>
#include <stdio.h>

namespace elmo_control {

static const unsigned SLEEP_TIME_MS = 1;


ElmoClient::ElmoClient(ethercat::EtherCatManager &manager, int slave_no)
  :manager_(manager), slave_no_(slave_no){}

void ElmoClient::writeOutputs(const ElmoOutput &output){
  uint8_t map[7] = {0};
  map[0] = (output.controlword) & 0x00ff;
  map[1] = (output.controlword >> 8) & 0x00ff;
  map[2] = (output.operation_mode) & 0x00ff;
  map[3] = (output.vel) & 0x00ff;
  map[4] = (output.vel >> 8) & 0x00ff;
  map[5] = (output.vel >> 16) & 0x00ff;
  map[6] = (output.vel >> 24) & 0x00ff;
  for(unsigned int i=0;i<7;i++)
    manager_.write(slave_no_, i, map[i]);
}

ElmoInput ElmoClient::readInputs() const{
  ElmoInput input;
  uint8_t map[17];
  for(unsigned int i=0;i<17;i++){
    map[i] = manager_.readInput(slave_no_, i);
  }
  input.position = *(uint32 *)(map+0);
  input.digital_inputs = *(uint32 *)(map+4);
  input.velocity = *(uint32 *)(map+8);
  input.status = *(uint16 *)(map+12);
  input.operation_mode = *(uint8 *)(map+14);
  input.current = *(uint16 *)(map+15);
  return input;
}

ElmoOutput ElmoClient::readOutputs() const{
  ElmoOutput output;
  uint8_t map[7];
  for(unsigned int i=0;i<7;i++){
    map[i] = manager_.readOutput(slave_no_, i);
  }
  output.controlword = *(uint16 *)(map+0);
  output.operation_mode = *(uint8 *)(map+2);
  output.vel = *(uint32 *)(map+3);
  return output;
}

void ElmoClient::reset(){
  ElmoInput input = readInputs();
  ElmoOutput output;
  memset(&output, 0x00, sizeof(ElmoOutput));
  output.controlword = 0x0080;
  output.operation_mode = 0x09;
  writeOutputs(output);
}


/*
PDS_OPERATION ElmoClient::getPDSOperation(const ElmoInput input) const{
  uint8 operation_mode = input.operation_mode;
  switch (operation_mode) {
  case 0:
    return NO_MODE_CHANGE;
    break;
  case 1:
    return PROFILE_POSITION_MODE;
    break;
  case 2:
    return VELOCITY_MODE;
    break;
  case 3:
    return PROFILE_VELOCITY_MODE;
    break;
  case 4:
    return TORQUE_PROFILE_MODE;
    break;
  case 6:
    return HOMING_MODE;
    break;
  case 7:
    return INTERPOLATED_POSITION_MODE;
    break;
  case 8:
    return CYCLIC_SYNCHRONOUS_POSITION_MODE;
    break;
  case 9:
    return CYCLIC_SYNCHRONOUS_VELOCITY_MODE;
    break;
  case 10:
    return CYCLIC_SYNCHRONOUS_TORQUE_MODE;
    break;
  }
}*/

PDS_CONTROL ElmoClient::getPDSControl(const ElmoInput input) const{
  uint16 statusword = input.status;
}

PDS_STATUS ElmoClient::getPDSStatus(const ElmoInput input) const{
  uint16 stausword = input.status;
  if(((stausword) & 0x004f) == 0x0000) //x0xx 0000
    return NOT_READY;
  else if(((stausword) & 0x004f) == 0x0040) //x1xx 0000
    return SWITCH_DISABLED;
  else if(((stausword) & 0x006f) == 0x0021) //x01x 0001
    return READY_SWITCH;
  else if(((stausword) & 0x006f) == 0x0023) //x01x 0011
    return SWITCHED_ON;
  else if(((stausword) & 0x006f) == 0x0027) //x01x 0111
    return OPERATION_ENABLED;
  else if(((stausword) & 0x006f) == 0x0007) //x00x 0111
    return QUICK_STOP;
  else if(((stausword) & 0x004f) == 0x000f) //x0xx 1111
    return FAULT_REACTION;
  else if(((stausword) & 0x004f) == 0x0008) //x0xx 1000
    return FAULT;
  else
    return UNKNOWN;
}



void ElmoClient::printPDSStatus(const ElmoInput input) const{
  printf("StatusWord(6041h): %04x\n ",input.status);
  switch (getPDSStatus(input)) {
  case NOT_READY:
    printf("Not ready to switch on\n");
    break;
  case SWITCH_DISABLED:
    printf("Switch on Disabled\n");
    break;
  case READY_SWITCH:
    printf("Ready to switch on\n");
    break;
  case SWITCHED_ON:
    printf("Switched on\n");
    break;
  case OPERATION_ENABLED:
    printf("Operation Enabled\n");
    break;
  case QUICK_STOP:
    printf("Quick stop active\n");
    break;
  case FAULT_REACTION:
    printf("Fault reaction active\n");
    break;
  case FAULT:
    printf("Fault\n");
    break;
  case UNKNOWN:
    printf("Unknown status %04x\n", input.status);
    break;
  }
  if(input.status & 0x0800)
    printf("Internal limit active\n");
}

/*
void ElmoClient::printPDSOperation(const ElmoInput input) const{
  printf("Mode of operation(6061h): %04x\n", input.operation_mode);
  switch (getPDSOperation(input)) {
  case NO_MODE_CHANGE:
    printf("NO mode change\n");
    break;
  case PROFILE_POSITION_MODE:
    printf("Profile position mode\n");
    break;
  case VELOCITY_MODE:
    printf("Velocity mode. NOt supported\n");
    break;
  case PROFILE_VELOCITY_MODE:
    printf("Profile velocity mode\n");
    break;
  case TORQUE_PROFILE_MODE:
    printf("Torque profile mode\n");
    break;
  case HOMING_MODE:
    printf("Homing mode\n");
    break;
  case INTERPOLATED_POSITION_MODE:
    printf("Interpolated position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_POSITION_MODE:
    printf("Cyclic synchronous position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
    printf("Cyclic synchronous velocity mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
    printf("Cyclic synchronous torque mode\n");
    break;
  default:
    printf("Reserved %04x\n", input.operation_mode);
    break;
  }
}*/

void ElmoClient::servoOn()
{
  ElmoInput input = readInputs();
  ElmoOutput output;
  memset(&output, 0x00, sizeof(ElmoOutput));
  int loop = 0;
  while (getPDSStatus(input) != OPERATION_ENABLED) {
    switch ( getPDSStatus(input) ) {
      case SWITCH_DISABLED:
  output.controlword = 0x0006; // move to ready to switch on
  break;
      case READY_SWITCH:
  output.controlword = 0x0007; // move to switched on
  break;
      case SWITCHED_ON:
  output.controlword = 0x000f; // move to operation enabled
  break;
      case OPERATION_ENABLED:
  break;
      default:
  printf("unknown status");
  return;
      }
    writeOutputs(output);
    usleep(SLEEP_TIME_MS*1000);
    input = readInputs();
    if (loop++ % 100 == 1) printPDSStatus(input);
  }
}


void ElmoClient::servoOff(){
  ElmoInput input = readInputs();
  printPDSStatus(input);
  ElmoOutput output;
  memset(&output, 0x00, sizeof(ElmoOutput));
  int loop = 0;
  while(getPDSStatus(input) != SWITCH_DISABLED){
    switch (getPDSStatus(input)) {
    case READY_SWITCH:
      output.controlword = 0x0000; //disable voltage
      break;
    case SWITCHED_ON:
      output.controlword = 0x0006; //shutdown
      break;
    case OPERATION_ENABLED:
      output.controlword = 0x0007; //disable operation
      break;
    default:
      printf("Unknown status");
      output.controlword = 0x0000; //disable operation
      break;
    }
    writeOutputs(output);
    usleep(SLEEP_TIME_MS * 1000);
    input = readInputs();
    if(loop++ %100 == 1) printPDSStatus(input);
  }
}

/*
void ElmoClient::setTorqueForEmergencyStop(double val){
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3511, 0x00, i16val);
}

void ElmoClient::setOverLoadLevel(double val){
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3512, 0x00, i16val);
}

void ElmoClient::setOverSpeedLevel(double val){
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3513, 0x00, i16val);
}

void ElmoClient::setMotorWorkingRange(double val){
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3514, 0x00, i16val);
}

void ElmoClient::setProfileVeclocity(uint32_t val){
  uint32_t u32val = (uint32_t)val;
  manager_.writeSDO<uint32_t>(slave_no_, 0x6081, 0x00, u32val);
}*/

}
