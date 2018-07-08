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
  fprintf(stderr, "Usage: elmo_client_velocity_control [options]\n");
  fprintf(stderr, " Available options\n");
  fprintf(stderr, "  -i, --interface NIC interface for the EtherCAT network\n");
  fprintf(stderr, "  -h, --print this message and exit\n");
}


int main(int argc, char *argv[]){
  //int operation_mode = 0x03;
  std::string ifname;
  printf("ELMO Simple test using SOEM (Simple Open EtherCAT master)\n");
  while(1){
    static struct option long_options[] = {
    {"help", no_argument, 0, 'h'},
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

    for(std::vector<elmo_control::ElmoClient*>::iterator it = clients.begin();it!=clients.end();++it){
      elmo_control::ElmoClient* client = (*it);
      client->reset();
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
        printf("Tick %8lu.%09lu", tick.tv_sec, tick.tv_nsec);
        printf("  pos: 0x%x, torque: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x \n",
               val.position, val.torque, val.velocity, val.status, val.operation_mode, val.current);

        client->servoOn();
        output.vel = (int16_t) (sin(i/100.)*(10000));
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
      client->printPDSStatus(input);
      client->servoOff();
    }
  }


  catch(...){help();}
  printf("End Program\n");
  return 0;
}
