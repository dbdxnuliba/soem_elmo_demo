#include <elmo_control/elmo_library.h>
#include <elmo_control/elmo_client.h>
#include <ethercat_manager/ethercat_manager.h>


#define PULSE_PER_REVOLUTE ( (1048576 / (2 * M_PI) ) * 101 ) // 20 bit / 101 reduction



namespace  elmo {
ElmoLibrary::ElmoLibrary(const std::string& ifname)
    :ifname_(ifname){

  clients_.resize(0);
  manager_ = new ethercat::EtherCatManager(ifname_);
  slave_no_ = manager_->getNumClients();
  for(int i=0;i<slave_no_;i++){
    clients_.push_back(new elmo_control::ElmoClient(*manager_, i+1));
  }
}

ElmoLibrary::~ElmoLibrary(){
  delete manager_;
}


void ElmoLibrary::initialize(){
  for(std::vector<elmo_control::ElmoClient*>::iterator it = clients_.begin();it != clients_.end();++it){
    elmo_control::ElmoClient* client = (*it);
    client->reset();
  }
}

std::vector<elmo_feedback> ElmoLibrary::readDataFromElmo(){
  std::vector<elmo::elmo_feedback> feeds;
  for(std::vector<elmo_control::ElmoClient*>::iterator it=clients_.begin();it!=clients_.end();++it){
      elmo_control::ElmoClient* client = (*it);
      elmo_control::ElmoInput val = client->readInputs();
      elmo_feedback feed;
      feed.vel_feedback = double_t(val.velocity/PULSE_PER_REVOLUTE);
      feed.pos_feedback = double_t(val.position/PULSE_PER_REVOLUTE);
      feed.effort_feedback = double_t(val.torque/PULSE_PER_REVOLUTE);
      feeds.push_back(feed);

  }
  return feeds;
}

void ElmoLibrary::writeToElmo(std::vector<double> &velocities){
  if(velocities.size() == slave_no_){
    for(int i=0;i<velocities.size();++i){
      elmo_control::ElmoClient* client = clients_[i];
      elmo_control::ElmoOutput output = client->readOutputs();
      client->servoOn();
      output.vel = uint32_t(velocities[i] * PULSE_PER_REVOLUTE)/100.;
      client->writeOutputs(output);
    }
  }
  else
    std::cerr<<"The size of the velocities should be same with number of clients \n";



}

int ElmoLibrary::getNumberofClients(){
  return slave_no_;
}


}//end namespace
