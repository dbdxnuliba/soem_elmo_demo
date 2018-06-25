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

static const unsigned THREAD_SLEEP_TIME = 1000; //1ms
static const unsigned EC_TIMEOUTMON = 500;
static const int NSEC_PER_SECOND = 1e+9;

//Keeping track of ticks
void timespecInc(struct timespec &tick, int nsec){
  tick.tv_nsec += nsec;
  while(tick.tv_nsec >= NSEC_PER_SECOND){
    tick.tv_nsec -= NSEC_PER_SECOND;
    tick.tv_sec++;
  }
}

//Handling Errors
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

//Main thread for sending and receiving data
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

} //end of namespace


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
  ec_slave[0].state = EC_STATE_INIT; //requesting init operational state for all slaves
  ec_writestate(0);//requesting init state for all slaves
  ec_close();//stop SOEM, close socket
  cycle_thread_.join();
}

int EtherCatManager::getNumClients() const{
  return num_clients_;
}

void EtherCatManager::getStatus(int slave_no, std::string &name, int &eep_man, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state,
               int &pdelay, int &hasdc, int &activeports, int &configadr) const{
  if(slave_no > ec_slavecount){
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slaveout(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  name = std::string(ec_slave[slave_no].name);
  eep_man = (int)ec_slave[slave_no].eep_man;
  eep_id = (int)ec_slave[slave_no].eep_id;
  eep_rev = (int)ec_slave[slave_no].eep_rev;
  obits = ec_slave[slave_no].Obits;
  ibits = ec_slave[slave_no].Ibits;
  state = ec_slave[slave_no].state;
  pdelay = ec_slave[slave_no].pdelay;
  hasdc = ec_slave[slave_no].hasdc;
  activeports = ec_slave[slave_no].activeports;
  configadr = ec_slave[slave_no].configadr;
}


void EtherCatManager::write(int slave_no, uint8_t channel, uint8_t value){
  boost::mutex::scoped_lock lock(iomap_mutex_);
  ec_slave[slave_no].outputs[channel] = value;
}

uint8_t EtherCatManager::readInput(int slave_no, uint8_t channel) const{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if(slave_no > ec_slavecount){
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if(channel * 8 >= ec_slave[slave_no].Ibits){
    fprintf(stderr, "ERROR : channel(%d) is larger than Input bits (%d)\n", channel * 8, ec_slave[slave_no].Ibits);
    exit(1);
  }
  return ec_slave[slave_no].inputs[channel];
}

uint8_t EtherCatManager::readOutput(int slave_no, uint8_t channel) const{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if(slave_no > ec_slavecount){
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if(channel * 8 >= ec_slave[slave_no].Obits){
    fprintf(stderr, "ERROR : channel(%d) is larger than Output bits (%d)\n", channel * 8, ec_slave[slave_no].Obits);
    exit(1);
  }
  return ec_slave[slave_no].outputs[channel];
}

template<typename T>
uint8_t EtherCatManager::writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const{
  int ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
  return ret;
}

template<typename T>
T EtherCatManager::readSDO(int slave_no, uint16_t index, uint8_t subidx) const{
  int ret, l;
  T val;
  l = sizeof(val);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
  if(ret <= 0){
    fprintf(stderr, "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
  }
  return val;
}


//doubt here
//#define IF_ELMO(_ec_slave) (((int)_ec_slave.eep_man == 0x066f) && ((((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0x5) || (((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0xD)))


bool EtherCatManager::initSoem(const std::string &ifname){
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if(name_size > sizeof(buffer)-1){
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);
  printf("Initializing etherCAT master\n");

  if(!ec_init(buffer)){
    fprintf(stderr, "Could not initialize ethercat driver\n");
    return false;
  }

  if(ec_config_init(FALSE) <= 0){
    fprintf(stderr, "No slaves found on %s\n", ifname_.c_str());
    return false;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);
  for(int cnt =1; cnt<=ec_slavecount;cnt++){
    printf("Man: %8.8x ID: %8.8x Rev: %8.8x \n",(int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
    num_clients_++;
  }//doubt here
  printf("Found %d ELMO Drivers\n", num_clients_);

  if(ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP){
    fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
    return false;
  }

  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++){
    /** set PDO mapping */
    int wkc = 0;
    int num_pdo = 0;

    //First setting 0 to both sync manager
    wkc +=ec_SDOwrite(cnt,0x1C12,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);
    wkc +=ec_SDOwrite(cnt,0x1C13,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //setting TPDO to 0
    wkc +=ec_SDOwrite(cnt,0x1A08,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //1st entry: position actual
    int32 position_actual = 0x60640020;
    wkc +=ec_SDOwrite(cnt,0x1A08,1, FALSE, sizeof(position_actual),&position_actual,EC_TIMEOUTRXM);

    //2nd entry: digital inputs
    int32 digital_inputs = 0x60FD0020;
    wkc +=ec_SDOwrite(cnt,0x1A08,2, FALSE, sizeof(digital_inputs),&digital_inputs,EC_TIMEOUTRXM);

    //3rd entry: velocity actual
    int32 velocity_actual = 0x606C0020;
    wkc +=ec_SDOwrite(cnt,0x1A08,3, FALSE, sizeof(velocity_actual),&velocity_actual,EC_TIMEOUTRXM);

    //4th entry: statusword
    int32 statusword = 0x60410010;
    wkc +=ec_SDOwrite(cnt,0x1A08,4, FALSE, sizeof(statusword),&statusword,EC_TIMEOUTRXM);

    //5th entry: mode of operation display
    int32 mode_of_op_disp = 0x60610008;
    wkc +=ec_SDOwrite(cnt,0x1A08,5, FALSE, sizeof(mode_of_op_disp),&mode_of_op_disp,EC_TIMEOUTRXM);

    //6th entry: current actual value
    int32 current_actual = 0x60780010;
    wkc +=ec_SDOwrite(cnt,0x1A08,6, FALSE, sizeof(current_actual),&current_actual,EC_TIMEOUTRXM);

    //setting total number of entries
    uint8_t t_num_of_entries = 6;
    wkc +=ec_SDOwrite(cnt,0x1A08,0, FALSE, sizeof(t_num_of_entries),&t_num_of_entries,EC_TIMEOUTRXM);


    //setting RPDO to 0
    wkc +=ec_SDOwrite(cnt,0x1608,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //1st entry: control word
    int32 controlword = 0x60400010;
    wkc +=ec_SDOwrite(cnt,0x1608,1, FALSE, sizeof(controlword),&controlword,EC_TIMEOUTRXM);

    //2nd entry: model of operation
    int32 mode_of_op = 0x60600008;
    wkc +=ec_SDOwrite(cnt,0x1608,2, FALSE, sizeof(mode_of_op),&mode_of_op,EC_TIMEOUTRXM);

    //3rd entry: target velocity
    int32 target_velocity = 0x60FF0020;
    wkc +=ec_SDOwrite(cnt,0x1608,3, FALSE, sizeof(target_velocity),&target_velocity,EC_TIMEOUTRXM);

    //setting total number of entries
    uint8_t r_num_of_entries = 3;
    wkc +=ec_SDOwrite(cnt,0x1608,0, FALSE, sizeof(r_num_of_entries),&r_num_of_entries,EC_TIMEOUTRXM);

    //Finally setting the mapping
    int16 rxpdo = 0x1608;
    wkc +=ec_SDOwrite(cnt,0x1C12,0x01, FALSE, sizeof(rxpdo),&rxpdo,EC_TIMEOUTRXM);

    int16 txpdo = 0x1A08;
    wkc +=ec_SDOwrite(cnt,0x1C13,0x01, FALSE, sizeof(txpdo),&txpdo,EC_TIMEOUTRXM);

    //Setting number of rxpdo
    int num_rpdo = 1;
    wkc +=ec_SDOwrite(cnt,0x1C12,0x00, FALSE, sizeof(num_rpdo),&num_rpdo,EC_TIMEOUTRXM);

    //Setting number of txpdo
    int num_tpdo = 1;
    wkc +=ec_SDOwrite(cnt,0x1C13,0x00, FALSE, sizeof(num_tpdo),&num_tpdo,EC_TIMEOUTRXM);
  }


  int iomap_size = ec_config_map(iomap_);
  printf("SOEM IOMap size: %d\n", iomap_size);
  ec_configdc();
  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP){
    fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
    return false;
  }

  //bringing drives to operational state
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  ec_writestate(0);
  int chk = 40;
  do{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
  }while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL){
    fprintf(stderr, "OPERATIONAL state not set, exiting\n");
    return false;
  }

  ec_readstate();
  for(int cnt=1;cnt<=ec_slavecount;cnt++){
    printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
           cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
    if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
    printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
           (ec_slave[cnt].activeports & 0x02) > 0 ,
           (ec_slave[cnt].activeports & 0x04) > 0 ,
           (ec_slave[cnt].activeports & 0x08) > 0 );
    printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
  }

  for(int cnt=1;cnt<=ec_slavecount;cnt++){
    int ret = 0, l;
    uint16_t sync_mode;
    uint32_t cycle_time;
    uint32_t minimum_cycle_time;
    uint32_t sync0_cycle_time;
    l = sizeof(sync_mode);
    ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
    l = sizeof(cycle_time);
    ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
    l = sizeof(minimum_cycle_time);
    ret += ec_SDOread(cnt, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
    l = sizeof(sync0_cycle_time);
    ret += ec_SDOread(cnt, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);
    printf("PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d\n", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
  }
  printf("\nFinished configuration successfully\n");
  return true;
}

template uint8_t EtherCatManager::writeSDO<char> (int slave_no, uint16_t index, uint8_t subidx, char value) const;
template uint8_t EtherCatManager::writeSDO<int> (int slave_no, uint16_t index, uint8_t subidx, int value) const;
template uint8_t EtherCatManager::writeSDO<short> (int slave_no, uint16_t index, uint8_t subidx, short value) const;
template uint8_t EtherCatManager::writeSDO<long> (int slave_no, uint16_t index, uint8_t subidx, long value) const;
template uint8_t EtherCatManager::writeSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx, unsigned char value) const;
template uint8_t EtherCatManager::writeSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx, unsigned int value) const;
template uint8_t EtherCatManager::writeSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx, unsigned short value) const;
template uint8_t EtherCatManager::writeSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx, unsigned long value) const;

template char EtherCatManager::readSDO<char> (int slave_no, uint16_t index, uint8_t subidx) const;
template int EtherCatManager::readSDO<int> (int slave_no, uint16_t index, uint8_t subidx) const;
template short EtherCatManager::readSDO<short> (int slave_no, uint16_t index, uint8_t subidx) const;
template long EtherCatManager::readSDO<long> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned char EtherCatManager::readSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned int EtherCatManager::readSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned short EtherCatManager::readSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned long EtherCatManager::readSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx) const;

}//end of ethercat namespace




