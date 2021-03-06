/**
 *  (c) 2018, Abhijit Makhal.
 *  Simple test to run the motor in velocity mode
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include <math.h>
#include <iostream>
#include <curses.h>


#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define EC_TIMEOUTMON 500

#define INITIAL_POS 0

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

struct VelOut {
    uint16 control;
    uint8 operation_mode;
    uint32 vel;

};
struct VelIn {
    int32 position;
    int32 torque;
    int32 velocity;
    uint16 status;
    int8 operation_mode;
    uint16 current;

};



template<typename T>
uint8_t readSDO(int slave_no, uint16_t index, uint8_t subidx, T value){
  int ret, l;
  //T val;
  l = sizeof(value);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &value, EC_TIMEOUTRXM);
  if(ret <= 0){
    fprintf(stderr, "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
  }
  return ret;
}

template<typename T>
uint8_t writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value){
  int ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
  return ret;
}



uint8_t readInput(int slave_no, uint8_t channel) {
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

uint8_t readOutput(int slave_no, uint8_t channel) {
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


VelIn readInputs(){
  VelIn input;
  uint8_t map[15];
  for(unsigned int i=0;i<15;i++){
    map[i] = readInput(1, i);
  }
  input.position = *(uint32 *)(map+0);
  input.torque = *(uint16 *)(map+4);
  input.velocity = *(uint32 *)(map+2);
  input.status = *(uint16 *)(map+10);
  input.operation_mode = *(uint8 *)(map+12);
  input.current = *(uint16 *)(map+14);
  return input;
}

void write(int slave_no, uint8_t channel, uint8_t value){
  ec_slave[slave_no].outputs[channel] = value;
}

void writeOutputs(const VelOut &output){
  uint8_t map[7] = {0};
  map[0] = (output.control) & 0x00ff;
  map[1] = (output.control >> 8) & 0x00ff;
  map[2] = (output.operation_mode) & 0x00ff;
  map[3] = (output.vel) & 0x00ff;
  map[4] = (output.vel >> 8) & 0x00ff;
  map[5] = (output.vel >> 16) & 0x00ff;
  map[6] = (output.vel >> 24) & 0x00ff;
  for(unsigned int i=0;i<7;i++)
    write(1, i, map[i]);
}





template uint8_t writeSDO<char> (int slave_no, uint16_t index, uint8_t subidx, char value);
template uint8_t writeSDO<int> (int slave_no, uint16_t index, uint8_t subidx, int value);
template uint8_t writeSDO<short> (int slave_no, uint16_t index, uint8_t subidx, short value);
template uint8_t writeSDO<long> (int slave_no, uint16_t index, uint8_t subidx, long value);
template uint8_t writeSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx, unsigned char value);
template uint8_t writeSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx, unsigned int value);
template uint8_t writeSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx, unsigned short value);
template uint8_t writeSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx, unsigned long value);

template uint8_t readSDO<char> (int slave_no, uint16_t index, uint8_t subidx, char value) ;
template uint8_t readSDO<int> (int slave_no, uint16_t index, uint8_t subidx, int value) ;
template uint8_t readSDO<short> (int slave_no, uint16_t index, uint8_t subidx, short value);
template uint8_t readSDO<long> (int slave_no, uint16_t index, uint8_t subidx, long value);
template uint8_t readSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx, unsigned char value);
template uint8_t readSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx, unsigned int value) ;
template uint8_t readSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx, unsigned short value) ;
template uint8_t readSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx, unsigned long value) ;




void simpletest(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */

      /** network discovery */
      if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         printf("Has CA? %s\n", ec_slave[1].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );

         /** CompleteAccess disabled for Elmo driver */
         ec_slave[1].CoEdetails ^= ECT_COEDET_SDOCA;

         ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

         /** set PDO mapping */
         int wkc = 0;
         int num_pdo = 0;

         //First setting 0 to both sync manager
         wkc +=ec_SDOwrite(1,0x1C12,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);
         wkc +=ec_SDOwrite(1,0x1C13,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

         //setting TPDO to 0
         wkc +=ec_SDOwrite(1,0x1A08,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

         //1st entry: position actual
         int32 position_actual = 0x60640020;
         wkc +=ec_SDOwrite(1,0x1A08,1, FALSE, sizeof(position_actual),&position_actual,EC_TIMEOUTRXM);

         //2nd entry: torque actual
         int32 torque_actual = 0x60770010;
         wkc +=ec_SDOwrite(1,0x1A08,2, FALSE, sizeof(torque_actual),&torque_actual,EC_TIMEOUTRXM);

         //3rd entry: velocity actual
         int32 velocity_actual = 0x606C0020;
         wkc +=ec_SDOwrite(1,0x1A08,3, FALSE, sizeof(velocity_actual),&velocity_actual,EC_TIMEOUTRXM);

         //4th entry: statusword
         int32 statusword = 0x60410010;
         wkc +=ec_SDOwrite(1,0x1A08,4, FALSE, sizeof(statusword),&statusword,EC_TIMEOUTRXM);

         //5th entry: mode of operation display
         int32 mode_of_op_disp = 0x60610008;
         wkc +=ec_SDOwrite(1,0x1A08,5, FALSE, sizeof(mode_of_op_disp),&mode_of_op_disp,EC_TIMEOUTRXM);

         //6th entry: current actual value
         int32 current_actual = 0x60780010;
         wkc +=ec_SDOwrite(1,0x1A08,6, FALSE, sizeof(current_actual),&current_actual,EC_TIMEOUTRXM);

         //setting total number of entries
         uint8_t t_num_of_entries = 6;
         wkc +=ec_SDOwrite(1,0x1A08,0, FALSE, sizeof(t_num_of_entries),&t_num_of_entries,EC_TIMEOUTRXM);


         //setting RPDO to 0
         wkc +=ec_SDOwrite(1,0x1608,0, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

         //1st entry: control word
         int32 controlword = 0x60400010;
         wkc +=ec_SDOwrite(1,0x1608,1, FALSE, sizeof(controlword),&controlword,EC_TIMEOUTRXM);

         //2nd entry: model of operation
         int32 mode_of_op = 0x60600008;
         wkc +=ec_SDOwrite(1,0x1608,2, FALSE, sizeof(mode_of_op),&mode_of_op,EC_TIMEOUTRXM);

         //3rd entry: target velocity
         int32 target_velocity = 0x60FF0020;
         wkc +=ec_SDOwrite(1,0x1608,3, FALSE, sizeof(target_velocity),&target_velocity,EC_TIMEOUTRXM);

         //setting total number of entries
         uint8_t r_num_of_entries = 3;
         wkc +=ec_SDOwrite(1,0x1608,0, FALSE, sizeof(r_num_of_entries),&r_num_of_entries,EC_TIMEOUTRXM);

         //Finally setting the mapping
         int16 rxpdo = 0x1608;
         wkc +=ec_SDOwrite(1,0x1C12,0x01, FALSE, sizeof(rxpdo),&rxpdo,EC_TIMEOUTRXM);

         int16 txpdo = 0x1A08;
         wkc +=ec_SDOwrite(1,0x1C13,0x01, FALSE, sizeof(txpdo),&txpdo,EC_TIMEOUTRXM);

         //Setting number of rxpdo
         int num_rpdo = 1;
         wkc +=ec_SDOwrite(1,0x1C12,0x00, FALSE, sizeof(num_rpdo),&num_rpdo,EC_TIMEOUTRXM);

         //Setting number of txpdo
         int num_tpdo = 1;
         wkc +=ec_SDOwrite(1,0x1C13,0x00, FALSE, sizeof(num_tpdo),&num_tpdo,EC_TIMEOUTRXM);




         /** if CA disable => automapping works */
         ec_config_map(&IOmap);

         printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               1, ec_slave[1].name, ec_slave[1].Obits, ec_slave[1].Ibits,
               ec_slave[1].state, ec_slave[1].pdelay, ec_slave[1].hasdc);


         printf("Slaves mapped, state to SAFE_OP.\n");

         int timestep = 700;

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);


         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         /** going operational */
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);

         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 40;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            wkc_count = 0;
            inOP = TRUE;

            /**
             * Drive state machine transistions
             *   0 -> 6 -> 7 -> 15
             */

            /* cyclic loop */
            int reachedInitial = 0;

            VelOut target;
            target.operation_mode = 0x09;
            target.control = 0x0080;

            while(1)
            {
              writeOutputs(target);
              VelIn val = readInputs();

               /** PDO I/O refresh */
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);


                    if(wkc >= expectedWKC)
                    {
                        printf("Processdata cycle %4d, WKC %d,", i, wkc);
                        printf("  pos: 0x%x, torque: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x ",
                               val.position, val.torque, val.velocity, val.status, val.operation_mode, val.current);

                        /** if in fault or in the way to normal status, we update the state machine */
                        switch(target.control){
                        case 0:
                            target.control = 6;
                            break;
                        case 6:
                            target.control = 7;
                            break;
                        case 7:
                            target.control = 15;
                            break;
                        case 128:
                            target.control = 0;
                            break;
                        default:
                            if(val.status >> 3 & 0x01){
                                //READ(0x1001, 0, buf8, "Error");
                                int8 error;
                                readSDO(1,0x1001, 0x00, error);
                                target.control = 128;
                            }
//                            break;
                        }


                        /** we wait to be in ready-to-run mode and with initial value reached */
                        if(reachedInitial == 0 /*&& val->position == INITIAL_POS */&& (val.status & 0x0fff) == 0x0237){
                            reachedInitial = 1;
                        }



                        if((val.status & 0x0fff) == 0x0237 && reachedInitial){
                            target.vel = 94770;

                       }

                        printf("  Target: 0x%x, control: 0x%x", target.vel, target.control);

                        printf("\r");
                        needlf = TRUE;
                    }
                    usleep(timestep);

                }
                inOP = FALSE;
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


            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
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
    int iret1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
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
