/**
 * \brief Example code for Simple Open EtherCAT master
 *
 * USage: slaveInfo [ifname]
 * Ifname is the NIC interface, eth0
 *
 * Shows the configured slave data
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>

int main(int argc, char *argv[]){
  ec_adaptert * adapter = NULL;
  printf("SOEM (Simple Open EtherCAT Master)\nSlaveInfo\n");
  if(argc>1){
    std::string ifname(argv[1]);
    ethercat::EtherCatManager manager(ifname);
  }
  else{
    printf("Usage: slaveInfo ifname [options]\nifname = eth0 for example\n");
    printf("Available adapters\n");
    adapter = ec_find_adapters();
    while(adapter != NULL){
      printf("Description: %s, Device to use for wpcap: %s\n", adapter->desc, adapter->name);
      adapter = adapter->next;
    }
  }
  printf("End program\n");
  return 0;
}
