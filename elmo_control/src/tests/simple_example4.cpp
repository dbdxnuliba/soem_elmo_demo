//controlling the motor by /cmd_vel from keyboard or joystick

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ethercat_manager/ethercat_manager.h>
#include <elmo_control/elmo_client.h>


class TeleopArg{
public:
  TeleopArg(ros::NodeHandle &node, std::string& ifname);
private:
  void teleCallback(const geometry_msgs::Twist& vel);
  void runMotor(double vel);
  ros::NodeHandle nh_;
  ros::Subscriber tele_sub_;
  ethercat::EtherCatManager* manager_;
  std::string ifname_;
  std::vector<elmo_control::ElmoClient *> clients_;
};

TeleopArg::TeleopArg(ros::NodeHandle &node, std::string& ifname) : nh_(node), ifname_(ifname){
  tele_sub_ = nh_.subscribe("cmd_vel", 1, &TeleopArg::teleCallback, this);

  //Calling the ethercat manager and elmo_client
  manager_ = new ethercat::EtherCatManager(ifname_);
  for(int i=0;i<manager_->getNumClients();i++){
    clients_.push_back(new elmo_control::ElmoClient(*manager_, i+1));
  }

  for(std::vector<elmo_control::ElmoClient*>::iterator it = clients_.begin();it!=clients_.end();++it){
    elmo_control::ElmoClient* client = (*it);
    client->reset();
  }
}

void TeleopArg::teleCallback(const geometry_msgs::Twist &vel){
  runMotor(vel.linear.x);
}

void TeleopArg::runMotor(double vel){
  for(int i=0;i<10000;i++){
    for(std::vector<elmo_control::ElmoClient*>::iterator it=clients_.begin();it!=clients_.end();++it){
      elmo_control::ElmoClient* client = (*it);
      elmo_control::ElmoInput val = client->readInputs();
      elmo_control::ElmoOutput output = client->readOutputs();
      client->servoOn();
      output.vel = vel * 40000;
      printf("  pos: 0x%x, digital input: 0x%x, vel: 0x%x, status: 0x%x, Mode: 0x%x Current: 0x%x \n",
             val.position, val.digital_inputs, val.velocity, val.status, val.operation_mode, val.current);
      client->writeOutputs(output);

    }
  }

  for(std::vector<elmo_control::ElmoClient*>::iterator it=clients_.begin();it!=clients_.end();++it){
    elmo_control::ElmoClient* client = (*it);
    elmo_control::ElmoInput val = client->readInputs();
    elmo_control::ElmoOutput output = client->readOutputs();
    client->servoOn();
    output.vel = vel * 0;
    client->writeOutputs(output);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_example4");
  std::cout<<"ELMO Simple test using SOEM (Simple Open EtherCAT master) and teleop "<<std::endl;;
  ros::NodeHandle nh;
  std::string ifname = "enp2s0";
  TeleopArg tele(nh, ifname);
  ros::spin();

  ROS_INFO("Hello world!");
}
