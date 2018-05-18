#include <ros/ros.h>
#include <stdio.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

#include <stdio.h>

//#include "soem.h"

char IOmap[4096];

void simpletest(char *ifname)
{
  int i, j, oloop, iloop;

  printf("Starting simple test\n");

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    printf("ec_init on %s succeeded.\n",ifname);
    /* find and auto-config slaves */

    if ( ec_config_init(FALSE) > 0 )
    {
      printf("%d slaves found and configured.\n",ec_slavecount);

      ec_config_map(&IOmap);

      printf("Slaves mapped, state to SAFE_OP.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
      if (oloop > 8) oloop = 8;
      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
      if (iloop > 8) iloop = 8;

      printf("Request operational state for all slaves\n");
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      /* wait for all slaves to reach OP state */
      ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
      if (ec_slave[0].state == EC_STATE_OPERATIONAL )
      {
        printf("Operational state reached for all slaves.\n");
        /* cyclic loop 10 times */
        for(i = 1; i <= 10; i++)
        {
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          printf("Processdata cycle %4d , O:", i);
          *(ec_slave[0].outputs)=0xff;
          for(j = 0 ; j < oloop; j++)
          {
            printf(" %2.2x", *(ec_slave[0].outputs + j));
          }
          printf(" I:");
          for(j = 0 ; j < iloop; j++)
          {
            printf(" %2.2x", *(ec_slave[0].inputs + j));
          }
          printf("\n");
        }
      }
      else
      {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for(i = 1; i<=ec_slavecount ; i++)
        {
          if(ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
              i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
      printf("Request safe operational state for all slaves\n");
      ec_slave[0].state = EC_STATE_SAFE_OP;
      /* request SAFE_OP state for all slaves */
      ec_writestate(0);
    }
    else
    {
      printf("No slaves found!\n");
    }
    printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n",ifname);
  }
}

int main(int argc, char *argv[])
{
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1)
  {
    /* start cyclic part */
    simpletest(argv[1]);
  }
  else
  {
    printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
  }

  printf("End program\n");
  return (0);
}
