#include <elmo_control/elmo_client.h>


namespace elmo_control {

static const unsigned SLEEP_TIME_MS = 1;


ElmoClient::ElmoClient(ethercat::EtherCatManager &manager, int slave_no)
  :manager_(manager), slave_no_(slave_no){}

/* Count the number of bits to be written
 * If it is 1 byte or 8 bit, shift once
 *          2 byte or 16 bit, shift twice
 *          3 byte or 24 bit, shift 3 times
 *          4 byte or 32 bit, shift 4 times
 * as for example, controlword 16 bits, shifted 2 times
 * and velocity 32 bits, shifted 4 times
 */
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

/* Count the number of bits to be read
 * similar to writing 8 bit read 1 byte
 *  16 bit read 2 bytes, 32 bit read 4 bytes
 */
ElmoInput ElmoClient::readInputs() const{
  ElmoInput input;
  uint8_t map[15];
  for(unsigned int i=0;i<15;i++){
    map[i] = manager_.readInput(slave_no_, i);
  }
  input.position = *(uint32_t *)(map+0);
  input.torque = *(uint16_t *)(map+4);
  input.velocity = *(uint32_t *)(map+2);
  input.status = *(uint16_t *)(map+10);
  input.operation_mode = *(uint8_t *)(map+12);
  input.current = *(uint16_t *)(map+14);
  return input;
}

ElmoOutput ElmoClient::readOutputs() const{
  ElmoOutput output;
  uint8_t map[7];
  for(unsigned int i=0;i<7;i++){
    map[i] = manager_.readOutput(slave_no_, i);
  }
  output.controlword = *(uint16_t *)(map+0);
  output.operation_mode = *(uint8_t *)(map+2);
  output.vel = *(uint32_t *)(map+3);
  return output;
}

void ElmoClient::reset(){
  ElmoInput input = readInputs();
  ElmoOutput output;
  memset(&output, 0x00, sizeof(ElmoOutput));
  output.controlword = 0x0080;
  output.operation_mode = 0x09;
  writeOutputs(output);
  std::cout<<"Reset called"<<std::endl;
}

/*
 * get current PDS status
 * set the enum and return it
 */
PDS_STATUS ElmoClient::getPDSStatus(const ElmoInput input) const{
  uint16_t stausword = input.status;
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


/*
 * print the current PDS status calling the getPDSStaus
 */
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
 * run through the state machines and set the drive to op enabled mode
 */
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

/*
 * run through the state machines and set the drive to switched off mode
 */
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


}
