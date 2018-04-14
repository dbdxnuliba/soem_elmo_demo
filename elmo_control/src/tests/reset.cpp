/**
 * \brief Example cide to reset SOEM
 *
 * usage : reset [ifname]
 * ifname is the NIC interface, e.g eth0
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <elmo_control/elmo_client.h>

int main(int argc, char *argv[]){
  printf("SOEM \n Simple test \n");
  if(argc>1){
    std::string ifname(argv[1]);
    ethercat::EtherCatManager manager(ifname);
    elmo_control::ElmoClient client(manager,1);
    client.reset();
  }
  else{
    printf("Usage: reset ifname \n ifname = eth0 for example\n");
  }
  printf("End program\n");
  return 0;
}
