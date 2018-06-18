/**
 *  (c) 2014, Manuel Vonthron - OPAL-RT Technologies, inc.
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
    uint32 vel;
    uint16 control;
};
struct VelIn {
    int32 position;
    int32 digital_inputs;
    int32 velocity;
    uint16 status;
    //int8 profile;
};

#define KEY_A 97
#define KEY_D 100

/**
 * helper macros
 */
#define READ(idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(1, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(1, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR()   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[1].state, ec_slave[1].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[1].ALstatuscode));    \
}

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
  //boost::mutex::scoped_lock lock(iomap_mutex_);
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
  //boost::mutex::scoped_lock lock(iomap_mutex_);
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
  uint8_t map[14];
  for(unsigned int i=0;i<14;i++){
    map[i] = readInput(1, i);
  }
  input.position = *(uint32 *)(map+0);
  input.digital_inputs = *(uint32 *)(map+4);
  input.velocity = *(uint32 *)(map+8);
  input.status = *(uint16 *)(map+12);

  return input;
}

void write(int slave_no, uint8_t channel, uint8_t value){
  //boost::mutex::scoped_lock lock(iomap_mutex_);
  ec_slave[slave_no].outputs[channel] = value;
}

void writeOutputs(const VelOut &output){
  uint8_t map[6] = {0};
  map[0] = (output.vel) & 0x00ff;
  map[1] = (output.vel >> 8) & 0x00ff;
  map[2] = (output.vel >> 16) & 0x00ff;
  map[3] = (output.vel >> 24) & 0x00ff;
  map[4] = (output.control) & 0x00ff;
  map[5] = (output.control >> 8) & 0x00ff;

  for(unsigned int i=0;i<6;i++)
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

    //struct VelIn *val;
    struct VelOut *target;

    int c = 0;


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
         /** opMode: 8  => Position profile */
         //WRITE(0x6060, 0, buf8, 9, "OpMode");
         uint8_t buf8_opmode;
         int l = sizeof(buf8_opmode);
         writeSDO(1, 0x6060, 0, 9);
         //ec_SDOread(1, 0x6061, 0x00, FALSE, &l, &buf8_opmode, EC_TIMEOUTRXM);

         readSDO(1, 0x6061, 0x00, buf8_opmode);

         //std::cout<<"The opmode display is: "<<buf8_opmode<<std::endl;

         //READ(0x6061, 0, buf8, "OpMode display");


         READ(0x1c12, 0, buf32, "rxPDO:0");
         READ(0x1c13, 0, buf32, "txPDO:0");

         READ(0x1c12, 1, buf32, "rxPDO:1");
         READ(0x1c13, 1, buf32, "txPDO:1");

//         WRITE(0x1c12, 1, buf16, 0x1601, "rxPDO");
//         WRITE(0x1c13, 1, buf16, 0x1A01, "txPDO");

        int32 ob2;int os;
         os=sizeof(ob2); ob2 = 0x16010001;
         ec_SDOwrite(1,0x1c12,0,TRUE,os,&ob2,EC_TIMEOUTRXM);
         //writeSDO(1, 0x1c12, 0, ob2);
         os=sizeof(ob2); ob2 = 0x1a030001;
         ec_SDOwrite(1,0x1c13,0, TRUE, os,&ob2,EC_TIMEOUTRXM);
         //writeSDO(1, 0x1c13, 0, ob2);

//         WRITE(0x1c12, 0, buf32, 0x16010001, "rxPDO");
//         WRITE(0x1c13, 0, buf32, 0x1A010001, "txPDO");

         READ(0x1c12, 0, buf32, "rxPDO:0");
         READ(0x1c13, 0, buf32, "txPDO:0");

         READ(0x1c12, 1, buf32, "rxPDO:1");
         READ(0x1c13, 1, buf32, "txPDO:1");


         /** if CA disable => automapping works */
         ec_config_map(&IOmap);

         /** let DC off for the time being */
//         ec_configdc();


         printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               1, ec_slave[1].name, ec_slave[1].Obits, ec_slave[1].Ibits,
               ec_slave[1].state, ec_slave[1].pdelay, ec_slave[1].hasdc);


         /** disable heartbeat alarm */
         //READ(0x10F1, 2, buf32, "Heartbeat?");
         //WRITE(0x10F1, 2, buf32, 1, "Heartbeat");


         //WRITE(0x60c2, 1, buf8, 2, "Time period");
         //WRITE(0x2f75, 0, buf16, 1, "Interpolation timeout");

         printf("Slaves mapped, state to SAFE_OP.\n");

         int timestep = 700;

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         /** old SOEM code, inactive
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 20) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 20) iloop = 8;*/

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         /** going operational */
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);


         READ(0x6083, 0, buf32, "Profile acceleration");
         READ(0x6084, 0, buf32, "Profile deceleration");
         READ(0x6085, 0, buf32, "Quick stop deceleration");

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
            READ(0x6041, 0, buf16, "*status word*");
            if(buf16 == 0x218){
                WRITE(0x6040, 0, buf16, 128, "*control word*"); usleep(100000);
                READ(0x6041, 0, buf16, "*status word*");
            }


            WRITE(0x6040, 0, buf16, 0, "*control word*"); usleep(100000);
            READ(0x6041, 0, buf16, "*status word*");

            WRITE(0x6040, 0, buf16, 6, "*control word*"); usleep(100000);
            READ(0x6041, 0, buf16, "*status word*");

            WRITE(0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
            READ(0x6041, 0, buf16, "*status word*");

            WRITE(0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
            READ(0x6041, 0, buf16, "*status word*");

   CHECKERROR();
            READ(0x1a0b, 0, buf8, "OpMode Display");

            READ(0x6061, 0, buf16, "*Mode of Operation*");
            std::cout<<"Mode of Operation now: : "<<(buf16 & 0x006f)<<std::endl;


            int reachedInitial = 0;

            READ(0x1001, 0, buf8, "Error");

            /* cyclic loop */
            //target = (struct VelOut *)(ec_slave[1].outputs);
            //val = (struct VelIn *)(ec_slave[1].inputs);
            //VelIn val = readInputs();


            //for(i = 1; i <= 100000; i++)
            while(1)
            {

              target = (struct VelOut *)(ec_slave[1].outputs);
              //VelOut target;
              //target.control = 0x0080;
              VelIn val = readInputs();
               /** PDO I/O refresh */
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
                        printf("Processdata cycle %4d, WKC %d,", i, wkc);
                        //printf("  pos: 0x%x, tor: 0x%x, stat: 0x%x, mode: 0x%x", val->position, val->digital_inputs, val->velocity, val->status);
                        printf("  pos: 0x%x, digital input: 0x%x, vel: 0x%x, mode: 0x%x", val.position, val.digital_inputs, val.velocity, val.status);

                        /** if in fault or in the way to normal status, we update the state machine */
                        switch(target->control){
                        case 0:
                            target->control = 6;
                            break;
                        case 6:
                            target->control = 7;
                            break;
                        case 7:
                            target->control = 15;
                            break;
                        case 128:
                            target->control = 0;
                            break;
                        default:
                            if(val.status >> 3 & 0x01){
                                READ(0x1001, 0, buf8, "Error");
                                target->control = 128;
                            }
//                            break;
                        }


                        /** we wait to be in ready-to-run mode and with initial value reached */
                        if(reachedInitial == 0 /*&& val->position == INITIAL_POS */&& (val.status & 0x0fff) == 0x0237){
                            reachedInitial = 1;
                        }



                        if((val.status & 0x0fff) == 0x0237 && reachedInitial){
                            //target->vel = (int16) (sin(i/1000.)*(10000));
                            target->vel = -94770;
                            std::cout<<"TARGET VELOCITY IS: "<<target->vel<<std::endl;
                             //writeOutputs(target);

                        }


                        //usleep(100000);




                        printf("  Target: 0x%x, control: 0x%x", target->vel, target->control);

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
            WRITE(0x10F1, 2, buf32, 0, "Heartbeat");
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

void ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(250);
    }
}

int main(int argc, char *argv[])
{
    int iret1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      //iret1 = pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
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
