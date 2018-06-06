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



bool simpletest(char *ifname){
  std::cout<<"Running simpletest"<<std::endl;

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
      ec_slave[0].state = EC_STATE_OPERATIONAL;

      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);

      /* request op state for all slaves */
      ec_writestate(0);

      /* wait for all slaves to reach op state */
      if(ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE*4) != EC_STATE_OPERATIONAL){
        std::cerr<<"Could not set EC_STATE_OPERATIONAL \n";
        return false;
      }
      std::cout<<"We are in op state now \n";

      if(ec_slave[0].state == EC_STATE_OPERATIONAL){
        std::cout<<"Operational state reached for all slaves \n";
        for(int i=1;i<100000;i++){
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          std::cout<<"Processdata cycle :"<<i;
          *(ec_slave[0].outputs) = 0xff; //0x00ff / 0xff = 255
          for(int j=0;j<oloop;j++){
            printf(" %2.2x", *(ec_slave[0].outputs + j));
            //std::cout<<" ,"<<*(ec_slave[0].outputs+j);
          }

          std::cout<<" I:";
          for(int j=0;j<iloop;j++){
            //printf(" %2.2x %d", *(ec_slave[0].inputs + j), *(ec_slave[0].inputs + j));
            printf(" %2.2x ", *(ec_slave[0].inputs + j));
            //std::cout<<" ,"<<*(ec_slave[0].inputs+j);
          }

          std::cout<<"\n";
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
