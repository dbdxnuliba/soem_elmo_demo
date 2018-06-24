#include<stdio.h>
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
  fprintf(stderr, "Usage: simple_example3 [options]\n");
  fprintf(stderr, " Available options\n");
  fprintf(stderr, "  -i, --interface NIC interface for the EtherCAT network\n");
  fprintf(stderr, "  -v, --velocity_profile mode Sample program using profile velocity mode \n");
  fprintf(stderr, "  -c, --cycliec_mode Sample program using cyclic synchronous velocity mode (Default)\n");
  fprintf(stderr, "  -h, --print this message and exit\n");
}


int main(int argc, char *argv[]){
  int operation_mode = 0x03;
  std::string ifname;
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
    if (c== -1) break;
    switch(c){
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

    for(std::vector<elmo_control::ElmoClient*>::iterator it = clients.begin(); it != clients.end(); ++it){
      elmo_control::ElmoClient* client = (*it);
      client->reset();
      client->servoOn();

      //get current velocity
      elmo_control::ElmoInput input = client->readInputs();
      int32 current_velocity = input.velocity;
      std::cout<<"current velocity is: "<<current_velocity<<std::endl;
    }

  }

  catch(...){help();}
  printf("End Program\n");
  return 0;
}
