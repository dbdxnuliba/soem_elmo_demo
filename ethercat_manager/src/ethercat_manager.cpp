#include <ethercat_manager/ethercat_manager.h>


#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace {

static const unsigned THREAD_SLEEP_TIME = 1000;
static const unsigned EC_TIMEOUTMON = 500;
static const int NSEC_PER_SECOND = 1e+9;

void timespecInc(struct timespec &tick, int nsec){
  tick.tv_nsec += nsec;
  while(tick.tv_nsec >= NSEC_PER_SECOND){
    tick.tv_nsec -= NSEC_PER_SECOND;
    tick.tv_sec++;
  }
}


void handleErrors(){
  ec_group[0].docheckstate = FALSE;
  ec_readstate();
  for(int slave = 1; slave <= ec_slavecount; slave++){
    if((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)){
      ec_group[0].docheckstate = TRUE;
      if(ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)){
        fprintf(stderr, "ERROR: slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state == EC_STATE_SAFE_OP){
        fprintf(stderr, "ERROR: slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state > 0){
        if(ec_reconfig_slave(slave, EC_TIMEOUTMON)){
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE: slave %d reconfigured\n",slave);
        }
      }
      else if(!ec_slave[slave].islost){
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if(!ec_slave[slave].state){
          ec_slave[slave].islost = TRUE;
          fprintf(stderr, "ERROR : slave %d lost\n", slave);
        }
      }
    }
  if(ec_slave[slave].islost){
    if(!ec_slave[slave].state){
      if(ec_recover_slave(slave, EC_TIMEOUTMON)){
        ec_slave[slave].islost = FALSE;
        printf("MESSAGE : slave %d recovered\n", slave);
      }
    }
    else{
      ec_slave[slave].islost = FALSE;
      printf("MESSAGE : slave %d found\n",slave);
    }
  }
 }
}

void cycleWorker(boost::mutex& mutex, bool& stop_flag){
  double period = THREAD_SLEEP_TIME * 1000;
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);
  while(!stop_flag){
    int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    int sent, wkc;
    {
      boost::mutex::scoped_lock lock(mutex);
      sent = ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
    }
    if(wkc < expected_wkc){
      handleErrors();
    }

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -
        (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if(overrun_time > 0.0){
      fprintf(stderr, " overrun: %f\n", overrun_time);
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    timespecInc(tick, period);
  }
}


}


namespace ethercat {

EtherCatManager::EtherCatManager(const std::string &ifname)
  :ifname_(ifname), num_clients_(0), stop_flag_(false){

  if(initSoem(ifname)){
    cycle_thread_ = boost::thread(cycleWorker, boost::ref(iomap_mutex_), boost::ref(stop_flag_));
  }
  else{
    throw EtherCatError("Could not initialize SOEM");
  }
}

EtherCatManager::~EtherCatManager(){
  stop_flag_ = true;
  ec_slave[0] = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
  cycle_thread_.join();
}

}




