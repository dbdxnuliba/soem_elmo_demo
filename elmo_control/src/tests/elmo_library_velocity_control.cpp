/**
  * Simple Test program to control multiple drives with different velocities
  * with elmo library
  */

#include <elmo_control/elmo_library.h>
#include <getopt.h>

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

int main(int argc, char **argv)
{
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

    double period = 4e+6;
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    timespecInc(tick, period);


    elmo::ElmoLibrary* elm = new elmo::ElmoLibrary(ifname);
    elm->initialize();
    int motors = elm->getNumberofClients();
    std::vector<double> vels;
    for(int i=0;i<motors;++i){
      vels.push_back((i+1)*0.5);
    }

    while(1){
      std::vector<elmo::elmo_feedback> feeds;
      feeds = elm->readDataFromElmo();
      printf("Tick %8lu.%09lu ", tick.tv_sec, tick.tv_nsec);
      for(int j=0;j<feeds.size();++j){
        std::cout<<"For motor["<<j+1<<"] "<< "Position: "<<feeds[j].pos_feedback<<
                   " velocity: "<<feeds[j].vel_feedback<<
                   " effort: "<<feeds[j].effort_feedback<<std::endl;

      }
      elm->writeToElmo(vels);
    }
    delete elm;
  }
  catch(...){help();}
  printf("End Program\n");
  return 0;
}
