#include <elmo_control/elmo_hardware_interface.h>
#include <getopt.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace elmo_control{

#define PULSE_PER_REVOLUTE ( (1048576 / (2 * M_PI) ) * 101 )

EtherCATJointControlInterface::EtherCATJointControlInterface(ethercat::EtherCatManager* manager, int slave_no,
      hardware_interface::JointStateInterface& jnt_stat, hardware_interface::VelocityJointInterface& jnt_cmd,
      int torque_for_emergency_stop, int over_load_level, int over_speed_level, double motor_working_range,
      int home_encoder_offset)
      : JointControllerInterface(slave_no, jnt_stat, jnt_cmd){
  std::string name;
  int eep_man, eep_id, eep_rev;
  int obits, ibits, state, pdelay, hasdc;
  int activeports, configadr;
  manager->getStatus(slave_no, name, eep_man, eep_id, eep_rev, obits, ibits, state, pdelay,hasdc,
                     activeports, configadr);
  std::stringstream ss;
  ss<<name<<"("<<configadr<<")";
  joint.hardware_id_ = ss.str();
  ROS_INFO("Initialize EtherCATJoint (%d) %s(man:%x, id:%x, rev:%x, port:%x, addr:%x)", slave_no, name.c_str(),
           eep_man, eep_id, eep_rev, activeports, configadr);
  int operation_mode = 0x09; // Setting operation mode (we have to change here)
  client_ = new ElmoClient(*manager, slave_no);
  ROS_INFO("Initialize EtherCATJoint (reset");
  client_->reset();

  ROS_INFO("Initialize EtherCATJoint (TorqueForEmergencyStop %d)",torque_for_emergency_stop);
  client_->setTorqueForEmergencyStop(torque_for_emergency_stop);
  ROS_INFO("Initialize EtherCATJoint (OverLoadLevel %d)",over_load_level);
  client_->setOverLoadLevel(over_load_level);
  ROS_INFO("Initialize EtherCATJoint (OverSpeedLevel %d)",over_speed_level);
  client_->setOverSpeedLevel(over_speed_level);
  ROS_INFO("Initialize EtherCATJoint (MotorWorkingRange %f)",motor_working_range);
  client_->setMotorWorkingRange(motor_working_range);
  //interpolationtimeperiod???

  ROS_INFO("Initialize EtherCATJoint (ServoOn)");
  client_->servoOn();

  //get current position
  ROS_INFO("Initialize EtherCATJoint (readInputs)");
  input_ = client_->readInputs();
  int32 current_velocity = input_.velocity_actual_value;
  ROS_INFO(" (Position Actual Value %d)", input_.position_actual_value);
  ROS_INFO(" (Velocity Actual Value %d)", input_.velocity_actual_value);
  ROS_INFO("Initialize EtherCATJoint (set target velocity)");

  memset(&output_, 0x00, sizeof(elmo_control::ELmoOutput));
  if(operation_mode == 0x03){
    output_.target_velocity = (current_velocity>0)?(current_velocity - 0x100000):(current_velocity+0x100000);
  }
  else{
    output_.target_velocity = current_velocity;
  }
  output_.controlword = 0x001f;
  output_.operation_mode = operation_mode;

  //client->setProfileVeclocity(0x20000000);
  client_->writeOutputs(output_);
  while ( ! (input_.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
        input_ = client_->readInputs();
  }
  ROS_INFO("Initialize EtherCATJoint (clear new set point)");
  output_.controlword   &= ~0x0010; // clear new-set-point (bit4)
  client_->writeOutputs(output_);

  if(abs(home_encoder_offset) > 3000000){
    ROS_WARN("Invalid large home_encoder_offset value: %d", home_encoder_offset);
    ROS_WARN("Please check your home_encoder_offset parameter is correct.");
    joint.home_encoder_offset_ = 0;
  }
  else{
    joint.home_encoder_offset_ = home_encoder_offset * 8;
  }
  ROS_INFO("Initialize EtherCATJoint .. done");
}


EtherCATJointControlInterface::~EtherCATJointControlInterface(){
  ROS_INFO_STREAM_NAMED("elmo","~EtherCATJointControlInterface()");
  shutdown();
  delete(client_);
}

void EtherCATJointControlInterface::shutdown(){
  ROS_INFO_STREAM_NAMED("elmo",joint.name_+" shutdown()");
  client_->printPDSStatus(input_);
  client_->printPDSOperation(input_);
  client_->reset();
  client_->servoOff();
}

void EtherCATJointControlInterface::read(){
  input_ = client_->readInputs();
  output_ = client_->readOutputs();
  joint.pos_ = int32_t(input_.position_actual_value - joint.home_encoder_offset_)/PULSE_PER_REVOLUTE;
  joint.vel_ = int32_t(input_.velocity_actual_value)/PULSE_PER_REVOLUTE;
  joint.position_actual_value_ = input_.position_actual_value;
  joint.velocity_actual_value_ = input_.velocity_actual_value;
}

void EtherCATJointControlInterface::write(){
  output_.target_velocity = uint32_t(joint.cmd_ * PULSE_PER_REVOLUTE);
  client_->writeOutputs(output_);
}

ElmoHardwareInterface::ElmoHardwareInterface(std::string ifname, bool in_simulation):manager_(NULL){
  if(in_simulation){
    ROS_INFO_STREAM_NAMED("elmo","ELMO hardware interface in simulation mode");
    for(int i=1;i<=4;i++){//set in yaml
      int torque_for_emergency_stop, over_load_level, over_speed_level;
      double motor_working_range;
      int home_encoder_offset;
      getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, home_encoder_offset);
      registerControl(new DummyJointControlInterface(i, joint_state_interface_, joint_velocity_interface_));
    }
  }
  else {
    manager_ = new ethercat::EtherCatManager(ifname);
    n_joints_ = manager_->getNumClients();
    if(n_joints_!=4){//set in yaml
      ROS_ERROR_STREAM_NAMED("elmo", "Elmo hardware interface expecting 4 joints");
    }
    for(int i=1;i<=n_joints_;i++){
      int torque_for_emergency_stop, over_load_level, over_speed_level;
      double motor_working_range;
      int home_encoder_offset;
      getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, home_encoder_offset);
      registerControl(new EtherCATJointControlInterface(manager_, i, joint_state_interface_,
                      joint_velocity_interface_,torque_for_emergency_stop, over_load_level, over_speed_level,
                      motor_working_range,home_encoder_offset));
    }
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_velocity_interface_);
}

ElmoHardwareInterface::~ElmoHardwareInterface(){
  shutdown();
}

void ElmoHardwareInterface::shutdown(){
  BOOST_FOREACH(JointControllerInterface* control, controls_){
    control->shutdown();
  }
  controls_.clear();
  if(manager_!=NULL){
    ROS_INFO_STREAM_NAMED("elmo","Delete Manager");
    delete(manager_);
  }
  manager_ = NULL;
}

void ElmoHardwareInterface::registerControl(JointControllerInterface* control){
  controls_.push_back(control);
}

bool ElmoHardwareInterface::read(const ros::Time time, const ros::Duration period){
  BOOST_FOREACH(JointControllerInterface* control, controls_){
    control->read();
  }
}

void ElmoHardwareInterface::write(const ros::Time time, const ros::Duration period){
  BOOST_FOREACH(JointControllerInterface* control, controls_){
    control->write();
  }
}

int ElmoHardwareInterface::getInputActualValueToStatus(std::vector<std::string> &joint_names,
                                                       std::vector<std::string> &hardware_ids,
                                                       std::vector<uint32> &position_actual_values,
                                                       std::vector<uint32> &velocity_actual_values){
  int n_joints = 0;
  BOOST_FOREACH(JointControllerInterface* control, controls_){
    std::string joint_name;
    std::string hardware_id;
    uint32 position_actual_value;
    uint32 velocity_actual_value;
    control->getInputActualValueToStatus(joint_name,hardware_id,position_actual_value,velocity_actual_value);
    joint_names.push_back(joint_name);
    hardware_ids.push_back(hardware_id);
    position_actual_values.push_back(position_actual_value);
    velocity_actual_values.push_back(velocity_actual_value);
    n_joints++;
  }
  return n_joints;
}

inline ros::Time ElmoHardwareInterface::getTime(){
  return ros::Time::now();
}

inline ros::Duration ElmoHardwareInterface::getPeriod(){
  return ros::Duration(0.001);
}

void ElmoHardwareInterface::getParamFromROS(int joint_no, int &torque_for_emergency_stop,
                                            int &over_load_level, int &over_speed_level,
                                            double &motor_working_range, int &home_encoder_offset){
  std::string joint_name("~joint"+boost::lexical_cast<std::string>(joint_no));
  ros::param::param<int>(joint_name+"/torque_for_emergency_stop",torque_for_emergency_stop,100);
  ros::param::param<int>(joint_name+"/over_load_level",over_load_level,50);
  ros::param::param<int>(joint_name+"/over_speed_level",over_speed_level,120);
  ros::param::param<double>(joint_name+"/motor_working_range",motor_working_range, 0.1);
  ROS_INFO_STREAM_NAMED("elmo",joint_name+"/torque_for_emergency_stop : "<<torque_for_emergency_stop);
  ROS_INFO_STREAM_NAMED("elmo",joint_name+"/over_load_level : "<<over_load_level);
  ROS_INFO_STREAM_NAMED("elmo",joint_name+"/over_speed_level : "<<over_speed_level);
  ROS_INFO_STREAM_NAMED("elmo",joint_name+"/motor_working_range : "<<motor_working_range);
  ros::param::param<int>(joint_name+"/home_encoder_offset",home_encoder_offset,0);
  ROS_INFO_STREAM_NAMED("elmo",joint_name+"/home_encoder_offset : "<<home_encoder_offset);
}

DummyJointControlInterface::DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat,
                                                       hardware_interface::VelocityJointInterface& jnt_cmd)
  : JointControllerInterface(slave_no, jnt_stat, jnt_cmd) {
  joint.cmd_ = joint.pos_ = joint.vel_ = joint.eff_ = joint.home_encoder_offset_ = 0;
}

void DummyJointControlInterface::read(){
  joint.pos_ = 0;
  joint.vel_ = joint.cmd_;
  joint.eff_ = 0;
  joint.position_actual_value_ = 0;
  joint.velocity_actual_value_ = 0;
}

void DummyJointControlInterface::write(){}



}

















