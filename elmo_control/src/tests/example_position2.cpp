#include <stdio.h>
#include <iostream>
#include <iomanip>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

char IOmap[4096];
int expectedWKC;
volatile int wkc;

struct Output {
    //uint16 torque;
    uint16 controlword;
};
struct Input {
    //int32 position;
    //int16 torque;
    uint16 statusword;
    //int8 profile;
};

void printStatus(int statusword){
  std::cout<<"current status is: "<<(statusword & 0x006f)<<std::endl;
  if((statusword & 0x006f) == 0)
    std::cout<<"Not ready \n";
  else if((statusword & 0x006f) == 40)
    std::cout<<"switch disabled \n";
  else if((statusword & 0x006f) == 21)
    std::cout<<"Ready to switch on \n";
  else if((statusword & 0x006f) == 23)
    std::cout<<"Switched ON here\n";
  else if((statusword & 0x006f) == 27)
    std::cout<<"operation enabled \n";
  else if((statusword & 0x006f) == 7)
    std::cout<<"quick stop \n";
  else if((statusword & 0x006f) == 15)
    std::cout<<"fault reaction \n";
  else if((statusword & 0x006f) == 8)
    std::cout<<"fault \n";
  else
    std::cout<<"Unknown \n";
}

#define WRITE(idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(1, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define READ(idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(1, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

bool simpletest(char *ifname){
  std::cout<<"Running simpletest"<<std::endl;

  uint32 buf32;
  uint16 buf16;
  uint8 buf8;

  struct Input *val;
  struct Output *target;

  /* initialise SOEM, bind socket to ifname */
  if(ec_init(ifname)){
    std::cout<<"ec_init on "<<ifname<<" succeded"<<std::endl;

    /* find and auto-config slaves */
    if(ec_config_init(FALSE) > 0){
      std::cout<<ec_slavecount<<" slaves found and configured"<<std::endl;

      /* how many slaves and their details */
      for(int cnt = 1;cnt<=ec_slavecount;cnt++){
        std::cout<<"Man: "<<std::setfill('0') << std::setw(8) <<std::hex<<int(ec_slave[cnt].eep_man)<<std::endl;
        std::cout<<"Id: "<<std::setfill('0') << std::setw(8) <<std::hex<<int(ec_slave[cnt].eep_id)<<std::endl;
        std::cout<<"Rev: "<<std::setfill('0') << std::setw(8) <<std::hex<<int(ec_slave[cnt].eep_rev)<<std::endl;
      }


      int8 num_pdo;
      int s = sizeof(num_pdo);
      ec_SDOread(1, 0x1c12, 0x00, FALSE, &s, &num_pdo, EC_TIMEOUTRXM);
      ec_SDOread(1, 0x1c13, 0x00, FALSE, &s, &num_pdo, EC_TIMEOUTRXM);

      ec_SDOread(1, 0x1c12, 0x01, FALSE, &s, &num_pdo, EC_TIMEOUTRXM);
      ec_SDOread(1, 0x1c13, 0x01, FALSE, &s, &num_pdo, EC_TIMEOUTRXM);



      int32 idx_rxpdo = 0x16020002;
      //int32 idx_rxpdo = 0x16020001;
      ec_SDOwrite(1,0x1c12,0,TRUE,sizeof(idx_rxpdo),&idx_rxpdo,EC_TIMEOUTRXM);

      int32 idx_txpdo = 0x1a020003;
      //int32 idx_txpdo = 0x1a020001;
      ec_SDOwrite(1,0x1c13,0x00, TRUE, sizeof(idx_txpdo),&idx_txpdo,EC_TIMEOUTRXM);


      uint32_t num_pdo2;
      int s2 = sizeof(num_pdo2);
      ec_SDOread(1, 0x1c12, 0x00, FALSE, &s2, &num_pdo2, EC_TIMEOUTRXM);
      ec_SDOread(1, 0x1c13, 0x00, FALSE, &s2, &num_pdo2, EC_TIMEOUTRXM);

      ec_SDOread(1, 0x1c12, 0x01, FALSE, &s2, &num_pdo2, EC_TIMEOUTRXM);
      ec_SDOread(1, 0x1c13, 0x01, FALSE, &s2, &num_pdo2, EC_TIMEOUTRXM);

      /*
      int wkc = 1;

      uint8_t SM_out = 0;
      uint8_t reg_size = 64;

      uint16_t syncManagerAdressRX = 0x1603;
      uint16_t syncManagerAdressTX = 0x1A03;

      //deactivate both sync-managers
      wkc &= ec_SDOwrite(1,0x1C12,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);
      wkc &= ec_SDOwrite(1,0x1C13,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);

      //set number of  process data items to zero
      wkc &= ec_SDOwrite(1,0x1603,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);
      wkc &= ec_SDOwrite(1,0x1A03,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);

      //set number of  process data items to 64
      wkc &= ec_SDOwrite(1,0x1603,0,FALSE,sizeof(reg_size),&reg_size,EC_TIMEOUTRXM);
      wkc &= ec_SDOwrite(1,0x1A03,0,FALSE,sizeof(reg_size),&reg_size,EC_TIMEOUTRXM);

      SM_out = 1;
      //write adresses for sync manager 1
      wkc &= ec_SDOwrite(1,0x1C12,1,FALSE,sizeof(syncManagerAdressRX),&syncManagerAdressRX,EC_TIMEOUTRXM);
      //activate it
      wkc &= ec_SDOwrite(1,0x1C12,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);

      //write adresses for sync manager 2
      wkc &= ec_SDOwrite(1,0x1C13,1,FALSE,sizeof(syncManagerAdressTX),&syncManagerAdressTX,EC_TIMEOUTRXM);
      //activate it
      wkc &= ec_SDOwrite(1,0x1C13,0,FALSE,sizeof(SM_out),&SM_out,EC_TIMEOUTRXM);*/



      int iomap_size = ec_config_map(&IOmap);
      std::cout<<"IOmap size is: "<<std::dec<<iomap_size<<std::endl;

      //first we have to set up the input output bits, then it will show up
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




      if(ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP){
        std::cerr<<"Could not set EC_STATE_PRE_OP \n";
        return false;
      }

      std::cout<<"We are in pre op state now \n";


      /* wait for all slaves to reach SAFE_OP state */
      if(ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP){
        std::cerr<<"Could not set EC_SAFE_STATE_OP \n";
        return false;
      }
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

      std::cout<<"We are in safe op state now \n";


      //This is the place we have to set all those complicated things
      int oloop = ec_slave[0].Obytes;
      std::cout<<"Size of oloop: "<<int(oloop)<<std::endl;
      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
      std::cout<<"Size of oloop: "<<int(oloop)<<std::endl;
      if (oloop > 8) oloop = 8;
      std::cout<<"Size of oloop: "<<int(oloop)<<std::endl;


      int iloop = ec_slave[0].Ibytes;
      std::cout<<"Size of iloop: "<<int(iloop)<<std::endl;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
      std::cout<<"Size of iloop: "<<int(iloop)<<std::endl;
      //if (iloop > 8) iloop = 8;
      std::cout<<"Size of iloop: "<<int(iloop)<<std::endl;

      std::cout<<"Requesting operational state for all slaves \n";
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;

      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);

      /* request op state for all slaves */
      ec_writestate(0);
      int chk = 40;
      /* wait for all slaves to reach OP state */
      do
      {
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      }
      while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

      /* wait for all slaves to reach op state */
      if(ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE*4) != EC_STATE_OPERATIONAL){
        std::cerr<<"Could not set EC_STATE_OPERATIONAL \n";
        return false;
      }
      std::cout<<"We are in op state now \n";



      if(ec_slave[0].state == EC_STATE_OPERATIONAL){
        std::cout<<"Operational state reached for all slaves \n";
        target = (struct Output *)(ec_slave[1].outputs);
        val = (struct Input *)(ec_slave[1].inputs);
        for(int i=1;i<100000;i++){
          ec_send_processdata();
          wkc = ec_receive_processdata(EC_TIMEOUTRET);
          if(wkc >= expectedWKC){
          READ(0x6041, 0, buf16, "*status word*");

          std::cout<<"buf16 is: "<<(buf16 & 0x006f)<<std::endl;
          printf("input: statusword 0x%x \n",val->statusword);
          std::cout<<"statusword: "<<(val->statusword & 0x006f)<<std::endl;
          printStatus(val->statusword);
          std::cout<<"controlword: "<<target->controlword<<std::endl;

          /*
          switch(target->controlword){
          case 0:
              target->controlword = 1;
              std::cout<<"Chaning from not ready to swtich on disabled \n";
              WRITE(0x6041, 0, buf16, 1, "control word");
              break;
          case 1:
              target->controlword = 2;
              std::cout<<"Chaning from switch disabled to ready to swtich on \n";
              WRITE(0x6040, 0, buf16, 2, "control word");
              break;
          case 2:
              target->controlword = 3;
              std::cout<<"Chaning from ready to swtich on to switch on \n";
              WRITE(0x6040, 0, buf16, 3, "control word");
              break;
          case 3:
              target->controlword = 4;
              std::cout<<"Chaning from switch on to op enabled \n";
              WRITE(0x6040, 0, buf16, 4, "control word");
              break;
          case 128:
              std::cout<<"Chaning from 128 to 0 \n";
              target->controlword = 0;
              WRITE(0x6040, 0, buf16, 0, "control word");
              break;
          default:
              if(val->statusword >> 3 & 0x01){
                  READ(0x1001, 0, buf8, "Error");
                  target->controlword = 128;
              }
          }*/
          if((buf16 & 0x006f) == 0){
            std::cout<<"Not ready \n";
            std::cout<<"Chaning from not ready to swtich on disabled \n";
            WRITE(0x6040, 0, buf16, 1, "control word");
          }

          else if((buf16 & 0x006f) == 40){
            std::cout<<"switch disabled \n";
            std::cout<<"Chaning from switch disabled to ready to swtich on \n";
            WRITE(0x6040, 0, buf16, 2, "control word");
          }

          else if((buf16 & 0x006f) == 21){
            std::cout<<"Ready to switch on \n";
            std::cout<<"Chaning from ready to swtich on to switch on \n";
            WRITE(0x6040, 0, buf16, 3, "control word");
          }

          else if((buf16 & 0x006f) == 23){
            std::cout<<"Switched ON \n";
            std::cout<<"Chaning from switch on to op enabled \n";
            WRITE(0x6040, 0, buf16, 4, "control word");
          }

          else if((buf16 & 0x006f) == 8){
            std::cout<<"in fault \n";
            std::cout<<"Chaning from fault to switch on disabled \n";
            WRITE(0x6040, 0, buf16, 15, "control word");
            READ(0x6041, 0, buf16, "*status word*");
            std::cout<<"buf16 is falut->switch on (8): "<<(buf16 & 0x006f)<<std::endl;
            printf("input: statusword 0x%x \n",val->statusword);
            std::cout<<"statusword: "<<(val->statusword & 0x006f)<<std::endl;
          }


          else
            std::cout<<"We are in an Unknown. dont know what to do now "<<(val->statusword & 0x006f)<<"\n";

        }

         std::cout<<"controlword: "<<target->controlword<<std::endl;
         printStatus(val->statusword);
      }
      }

      else{
        std::cout<<"Not all slaves reached operational state \n";
        ec_readstate();
        for(int i = 1; i<=ec_slavecount ; i++){
          if(ec_slave[i].state != EC_STATE_OPERATIONAL){
            std::cout<<"Slave "<<i<<" State = "<<ec_slave[i].state<<
                 "StatusCode = "<<ec_slave[i].ALstatuscode<<" : "<<ec_ALstatuscode2string(ec_slave[i].ALstatuscode);
          }
        }
      }

      std::cout<<"Request safe operational state for all slaves\n";
      ec_slave[0].state = EC_STATE_SAFE_OP;
      /* request SAFE_OP for all slaves */
      ec_writestate(0);
    } // ending if(ec_config_init(FALSE) > 0)

    else{
      std::cout<<"No slaves found!\n";
    }

    std::cout<<"End simple test, close socket\n";
    ec_close();
  }// ending if(ec_init(ifname))

  else {
    std::cout<<"No socket connection on "<<ifname<<" Execute as root\n";
  }
}


int main(int argc, char **argv)
{
  std::cout<<"I am an ELMO position control program"<<std::endl;
  if(argc > 1){
    //start cyclic part
    simpletest(argv[1]);
  }
  else{
    std::cout<<"Usage : example_position ifname = enp2s0"<<std::endl;
  }
  std::cout<<"End Program"<<std::endl;
  return 0;
}
