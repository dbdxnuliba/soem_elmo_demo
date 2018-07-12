#include <elmo_control/elmo_library.h>
#include <elmo_control/elmo_client.h>
#include <ethercat_manager/ethercat_manager.h>

#define PULSE_PER_REVOLUTE ( (1048576 / (2 * M_PI) ) * 101 ) // 20 bit / 101 reduction

namespace  elmo {
ElmoLibrary::ElmoLibrary(const std::string& ifname)
    :ifname_(ifname){

  feeds_.resize(0);
  clients_.resize(0);
  ethercat::EtherCatManager manager(ifname_);
  slave_no_ = manager.getNumClients();
  for(int i=0;i<slave_no_;i++){
    clients_.push_back(new elmo_control::ElmoClient(manager, i+1));
  }

  //for(std::vector<elmo_control::ElmoClient*>::iterator it = clients_.begin();it != clients_.end();++it){
    //elmo_control::ElmoClient* client = (*it);
    //client->reset();
    //elmo_control::ElmoInput val = client->readInputs();
    //printf("  pos: 0x%x, torque: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x \n",
                   //val.position, val.torque, val.velocity, val.status, val.operation_mode, val.current);
  //}
  //std::vector<elmo::elmo_feedback> feeds = readDataFromElmo();
}

void ElmoLibrary::initialize(){
  for(std::vector<elmo_control::ElmoClient*>::iterator it = clients_.begin();it != clients_.end();++it){
    elmo_control::ElmoClient* client = (*it);
    client->reset();
    //elmo_control::ElmoInput val = client->readInputs();
    //printf("  pos: 0x%x, torque: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x \n",
                   //val.position, val.torque, val.velocity, val.status, val.operation_mode, val.current);
  }
  std::cout<<"I am initialized"<<std::endl;
}

std::vector<elmo_feedback> ElmoLibrary::readDataFromElmo(){
  std::vector<elmo::elmo_feedback> feeds;
  for(std::vector<elmo_control::ElmoClient*>::iterator it=clients_.begin();it!=clients_.end();++it){
      elmo_control::ElmoClient* client = (*it);
      elmo_control::ElmoInput val = client->readInputs();


      printf("  pos: 0x%x, torque: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x \n",
             val.position, val.torque, val.velocity, val.status, val.operation_mode, val.current);

      std::cout<<"Reading"<<std::endl;


   //}
  }
  return feeds;
  /*elmo_feedback feed;
  feed.vel_feedback_ = double_t(val.velocity/PULSE_PER_REVOLUTE);
  feed.pos_feedback_ = double_t(val.position/PULSE_PER_REVOLUTE);
  feed.effort_feedback_ = double_t(val.torque/PULSE_PER_REVOLUTE);
  feeds.push_back(feed);*/
}



}//end namespace
