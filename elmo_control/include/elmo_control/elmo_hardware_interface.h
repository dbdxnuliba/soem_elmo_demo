#ifndef ELMO_HARDWARE_INTERFACE_H
#define ELMO_HARDWARE_INTERFACE_H

#include <ros/ros.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ethercat_manager/ethercat_manager.h>
#include <elmo_control/elmo_client.h>

namespace elmo_control {

struct JointData{
  std::string name_;
  std::string hardware_id_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
  int home_encoder_offset_;
  uint32 position_actual_value_;
  uint32 velocity_actual_value_;
};

class JointControllerInterface{
public:
  JointControllerInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::VelocityJointInterface& jnt_cmd){
    std::stringstream ss;
    ss<<" joint"<<slave_no;
    joint.name_ = ss.str();
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.eff_);
    jnt_stat.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_stat.getHandle(joint.name_), &joint.cmd_);
    jnt_cmd.registerHandle(vel_handle);
  }

  ~JointControllerInterface() {};
  virtual void read() = 0;
  virtual void write() = 0;
  virtual void shutdown() = 0;

  void getInputActualValueToStatus(std::string &joint_name, std::string &hardware_id,
                                   uint32 &position_actual_value, uint32 &velocity_actual_value){
    joint_name = joint.name_;
    hardware_id = joint.hardware_id_;
    position_actual_value = joint.position_actual_value_;
    velocity_actual_value = joint.velocity_actual_value_;
  }

protected:
  JointData joint;

};

class EtherCATJointControlInterface : public JointControllerInterface{
public:
  EtherCATJointControlInterface(ethercat::EtherCatManager* manager, int slave_no,
                                hardware_interface::JointStateInterface& jnt_stat,
                                hardware_interface::VelocityJointInterface& jnt_cmd,
                                int torque_for_emergency_stop, int over_load_level, int over_speed_level,
                                double motor_working_range, int home_encoder_offset);
  ~EtherCATJointControlInterface();
  void read();
  void write();
  void shutdown();

private:
  elmo_control::ElmoClient* client_;
  elmo_control::ElmoInput input_;
  elmo_control::ELmoOutput output_;
};

class ElmoHardwareInterface : public hardware_interface::RobotHW{
public:
  ElmoHardwareInterface(std::string ifname, bool in_simulation = false);
  ~ElmoHardwareInterface();
  void registerControl(JointControllerInterface*);
  bool read(const ros::Time time, const ros::Duration period);
  void write(const ros::Time time, const ros::Duration period);
  void shutdown();
  ros::Time getTime();
  ros::Duration getPeriod();
  int getInputActualValueToStatus(std::vector<std::string>& joint_names,
                                  std::vector<std::string>& hardware_ids,
                                  std::vector<uint32>& position_actual_values,
                                  std::vector<uint32>& velocity_actual_values);
  void getParamFromROS(int joint_no, int& torque_for_emergency_stop, int& over_load_level,
                       int& over_speed_level, double& motor_working_range,
                       int& home_encoder_offset);
private:
  ros::NodeHandle nh_;
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  typedef std::vector<JointControllerInterface*> JointControllerContainer;
  JointControllerContainer controls_;
  ethercat::EtherCatManager* manager_;
  unsigned int n_joints_;
};

class DummyJointControlInterface : public JointControllerInterface{
public:
  DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat,
                             hardware_interface::VelocityJointInterface& jnt_cmd);
  ~DummyJointControlInterface() {};
  void read();
  void write();
  void shutdown() {};
};



}

#endif // ELMO_HARDWARE_INTERFACE_H
