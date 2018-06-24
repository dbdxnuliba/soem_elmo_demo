/**
  * \brief Example for for SOEM
  *
  * Usage : simple_example -i [ifname]
  * ifname is NIC interface, e.g eth0
  *
  */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <elmo_control/elmo_client.h>
#include <getopt.h>
#include <time.h>

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

void timespecInc(struct  timespec &tick, int nsec){
  tick.tv_nsec += nsec;
  while(tick.tv_nsec >= NSEC_PER_SECOND){
    tick.tv_nsec -= NSEC_PER_SECOND;
    tick.tv_sec++;
  }
}

void help(){
  fprintf(stderr, "Usage: simple_example [options]\n");
  fprintf(stderr, " Available options\n");
  fprintf(stderr, "  -i, --interface NIC interface for the EtherCAT network\n");
  fprintf(stderr, "  -v, --velocity_profile mode Sample program using profile velocity mode \n");
  fprintf(stderr, "  -c, --cycliec_mode Sample program using cyclic synchronous velocity mode (Default)\n");
  fprintf(stderr, "  -h, --print this message and exit\n");
}

int main(int argc, char *argv[]){
  int operation_mode = 0x03;
  std::string ifname = "enp2s0";
  printf("ELMO Simple test using SOEM (Simple Open EtherCAT master)\n");
  while(1){
    static struct option long_options[] = {
    {"help", no_argument, 0, 'h'},
    {"profile_velocity_mode", no_argument, 0, 'v'},
    {"cyclic_mode", no_argument, 0, 'c'},
    {"interface", no_argument, 0, 'i'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hpci:", long_options, &option_index);
    if(c==-1)break;
    switch (c) {
    case 'h':
      help();
      exit(0);
      break;
    case 'v':
      operation_mode = 0x03;
      break;
    case 'c':
      operation_mode = 0x09;
      break;
    case 'i':
      ifname = optarg;
      break;
    }
  }
  try{
    ethercat::EtherCatManager manager(ifname);
    std::vector<elmo_control::ElmoClient *> clients;
    for(int i=0;i<manager.getNumClients();i++){
      clients.push_back(new elmo_control::ElmoClient(manager, i+1));
    }

    std::cout<<"There are: "<<manager.getNumClients()<<" of clinets"<<std::endl;
    for(std::vector<elmo_control::ElmoClient*>::iterator it = clients.begin();it!=clients.end();++it){
      elmo_control::ElmoClient* client = (*it);

      client->reset();

      //client->setTorqueForEmergencyStop(100);
      //client->setOverLoadLevel(50);
      //client->setOverSpeedLevel(120);
      //client->setMotorWorkingRange(0.1);

      client->servoOn();

      //get current velocity
      elmo_control::ElmoInput input = client->readInputs();
      int32 current_velocity = input.velocity;
      std::cout<<"current velocity is: "<<current_velocity<<std::endl;

      //set target velocity
      elmo_control::ElmoOutput output;
      //std::cout<<"target velocity: "<<output.target_velocity<<std::endl;
      memset(&output, 0x00, sizeof(elmo_control::ElmoOutput));

      /*if(operation_mode == 0x03){
        std::cout<<"target: "<<current_velocity - 0x100000<<std::endl;
        output.target_velocity = (current_velocity>0)?(current_velocity - 0x100000):(current_velocity+0x100000);
        std::cout<<"target velocity: "<<output.target_velocity<<std::endl;
      }
      else{
        std::cout<<"In else here"<<std::endl;
        output.target_velocity = current_velocity;}*/

      //output.controlword = 0x001f;
      output.operation_mode = 0x03;
      //output.vel = 94770;

      //client->setProfileVeclocity(0x20000000);
      /*client->writeOutputs(output);
      while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
        input = client->readInputs();
      }
      output.controlword &= ~0x0010; // clear new-set-point (bit4)
      client->writeOutputs(output);
      printf("target velocity = %08x\n", output.target_velocity);*/
    }


    double period = 4e+6;
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    timespecInc(tick, period);

    for(int i=0;i<=5000;i++){
      for(std::vector<elmo_control::ElmoClient*>::iterator it=clients.begin();it!=clients.end();++it){
        elmo_control::ElmoClient* client = (*it);
        elmo_control::ElmoInput val = client->readInputs();
        elmo_control::ElmoOutput output = client->readOutputs();
        if(i%10 == 0){
          //printf("ctrl %04x, status %04x, op_mode = %2d, pos = %08x, vel = %08x\n",
                 //output.controlword, input.status, input.operation_mode, input.position, input.velocity);
          /*if(input.status & 0x0400){
            printf("target reached\n");
            break;
          }*/
          printf("Tick %8lu.%09lu\n", tick.tv_sec, tick.tv_nsec);
          printf("Input:\n");
          printf("  pos: 0x%x, digital input: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x ",
                 val.position, val.digital_inputs, val.velocity, val.status, val.operation_mode, val.current);

          //printf(" 6041h %08x : StatusWord\n", input.statusword);
          //printf(" 6061h %08x : Modes of Operation Display\n", input.operation_mode);
          //printf(" 22A2h %08x : Drive Temp\n", input.drivetemp);
          //printf(" 6078h %08x : Current Actual Value\n", input.current_actual_value);
          //printf(" 6064h %08x : Position Actual Value\n", input.position_actual_value);
          //printf(" 606Ch %08x : Velocity Actual Value\n", input.velocity_actual_value);
          //printf(" 2205.1h %08x : Analog input 1\n", input.analog_input_1);
          //printf(" 2206h %08x : 5V DC supply\n", input.dc_supply_5v);
          //printf(" 6079h %08x : DC link Circuit Voltage\n", input.dc_link_circuit_voltage);
          //printf(" 60FDh %08x : Digital Input\n", input.digital_input);
          //printf("Output:\n");
          //printf(" 6040h %08x : ControlWord\n", output.controlword);
          //printf(" 6060h %08x : Modes of Operation\n", output.operation_mode);
          //printf(" 60FFh %08x : Target Velocity\n", output.target_velocity);
        }
        if(output.operation_mode = 0x03){
          //output.vel = 0x80000*sin(i/200.0);
          output.vel = (int16) (sin(i/100.)*(10000));
        }
        client->writeOutputs(output);
      }

      timespecInc(tick, period);
      struct timespec before;
      clock_gettime(CLOCK_REALTIME, &before);
      double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) - (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
      if(overrun_time>0.0){
        fprintf(stderr, " overrun: %f", overrun_time);
      }
      clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }

    for(std::vector<elmo_control::ElmoClient*>::iterator it = clients.begin(); it!=clients.end();++it){
      elmo_control::ElmoClient* client = (*it);
      elmo_control::ElmoInput input = client->readInputs();
      //client->printPDSStatus(input);
      //client->printPDSOperation(input);
      //client->servoOff();
    }

  }catch(...){help();}
  printf("End Program\n");
  return 0;
}


























