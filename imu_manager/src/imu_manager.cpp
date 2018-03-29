#include <imu_manager/imu_manager.h>


ImuManager::ImuManager(ros::NodeHandle &node, const std::string &stop_service,
                       const std::string &reload_service, const std::string imu_topic,
                       int baudRate, int timeOut, const std::string frame_id):
  nh_(node), baudRate_(baudRate), timeOut_(timeOut), running_(true),
  frame_id_(frame_id)
{
  pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1000);
  reload_service_ = nh_.advertiseService(reload_service, &ImuManager::reload_callback, this);
  stop_service_ = nh_.advertiseService(stop_service, &ImuManager::stop_callback, this);

}

ros::Time ImuManager::getTime() const{
  return ros::Time::now();
}

ros::Duration ImuManager::getPeriod() const{
  return ros::Duration(0.01);
}

bool ImuManager::reload_callback(std_srvs::Empty::Request &, std_srvs::Empty::Response &){
  running_ = true;
  return true;
}

bool ImuManager::stop_callback(std_srvs::Empty::Request &, std_srvs::Empty::Response &){
  running_ = false;
  return true;
}

void ImuManager::read(){
  ROS_INFO_STREAM("reading");
}

void ImuManager::write(){
  ROS_INFO_STREAM("publishing");
}
