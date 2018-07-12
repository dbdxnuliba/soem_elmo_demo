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
    elmo::ElmoLibrary elm(ifname);
    elm.initialize();
    //std::vector<elmo::elmo_feedback> feeds = elm.readDataFromElmo();
    //std::cout<<"Position: "<<feeds[0].pos_feedback_<<std::endl;

    /*for(int i=0;i<=5000;i++){
      std::vector<elmo::elmo_feedback> feeds;
      feeds = elm.readDataFromElmo();

      for(int j=0;j<feeds.size();++j){
        std::cout<<"Position: "<<feeds[j].pos_feedback_<<std::endl;
      }

    }*/


  }
  catch(...){help();}
  printf("End Program\n");
  return 0;
}
