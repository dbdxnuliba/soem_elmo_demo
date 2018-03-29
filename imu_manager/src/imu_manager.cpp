#include <imu_manager/imu_manager.h>


ImuManager::ImuManager(ros::NodeHandle &node, const std::string &stop_service,
                       const std::string &reload_service, const std::string imu_topic,
                       const int baudRate,const int rate, const std::string frame_id, const std::string sensor_port):
  nh_(node), baudRate_(baudRate), rate_(rate), running_(true),
  frame_id_(frame_id), sensor_port_(sensor_port)
{
  //publisher and services
  pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1000);
  reload_service_ = nh_.advertiseService(reload_service, &ImuManager::reload_callback, this);
  stop_service_ = nh_.advertiseService(stop_service, &ImuManager::stop_callback, this);

  //serial port settings
  ROS_INFO("Connecting to : %s @ %d baudRate", sensor_port_.c_str(), baudRate_);


  //Connecting to VnSensor object
  try{
    vs_.connect(sensor_port_, baudRate_);
  }
  //catching an exception when the sensor is not connected
  catch(const std::exception& e){
    ROS_ERROR("The IMU sensor is not connected. Please check your sensor");
    exit(1);
  }


  //Obtaining sensor's model number
  std::string mn = vs_.readModelNumber();
  ROS_INFO("Model Number: %s", mn.c_str());

  //Setting output data frequency
  vs_.writeAsyncDataOutputFrequency(rate_);

  //configue binary output message
  BinaryOutputRegister bor(
      ASYNCMODE_PORT1,
      1000 / rate_,  // update rate [ms]
      COMMONGROUP_TIMESTARTUP | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
      TIMEGROUP_NONE,
      IMUGROUP_NONE,
      GPSGROUP_NONE,
      ATTITUDEGROUP_NONE,
  INSGROUP_NONE);

  vs_.writeBinaryOutput1(bor);


  //ros_control hardware_interfacing
  hardware_interface::ImuSensorHandle::Data data;
  data.name = "imu_data";
  data.frame_id = frame_id_;
  data.orientation = orientation_;
  data.orientation_covariance = orientation_covariance_;
  data.angular_velocity = ang_vel_;
  data.angular_velocity_covariance = ang_vel_covariance_;
  data.linear_acceleration = lin_acc_;
  data.linear_acceleration_covariance = lin_acc_covariance_;

  imu_sensor_handle_ = hardware_interface::ImuSensorHandle(data);
  imu_sensor_interface_.registerHandle(imu_sensor_handle_);
  registerInterface(&imu_sensor_interface_);
}

ImuManager::~ImuManager(){
  vs_.unregisterAsyncPacketReceivedHandler();
  vs_.disconnect();
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
  vs_.registerAsyncPacketReceivedHandler(NULL, (void(*)(void*, vn::protocol::uart::Packet&, size_t))&ImuManager::BinaryAsyncMessageReceived);
}

void ImuManager::BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index){
  if(p.type() == Packet::TYPE_BINARY){
    if (!p.isCompatible(
          COMMONGROUP_TIMESTARTUP | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
          TIMEGROUP_NONE,
          IMUGROUP_NONE,
          GPSGROUP_NONE,
          ATTITUDEGROUP_NONE,
          INSGROUP_NONE))
          // Not the type of binary packet we are expecting.
    return;
    //Unpack the packet
    vec4f q = p.extractVec4f();
    vec3f ar = p.extractVec3f();
    vec3d lla = p.extractVec3d();
    orientation_[0] = double(q[0]);
    orientation_[1] = double(q[1]);
    orientation_[2] = double(q[2]);
    ang_vel_[0] = double(ar[0]);
    ang_vel_[1] = double(ar[1]);
    ang_vel_[2] = double(ar[2]);
    lin_acc_[0] = double(lla[0]);
    lin_acc_[1] = double(lla[1]);
    lin_acc_[2] = double(lla[2]);
  }
}

void ImuManager::write(){
  ROS_INFO_STREAM("publishing");
  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.frame_id = frame_id_;
  const double* orientation = imu_sensor_handle_.getOrientation();
  imu_data_.orientation.x = orientation[0];
  imu_data_.orientation.y = orientation[1];
  imu_data_.orientation.z = orientation[2];
  imu_data_.orientation.w = orientation[3];
  const double* angular_velocity = imu_sensor_handle_.getAngularVelocity();
  imu_data_.angular_velocity.x = angular_velocity[0];
  imu_data_.angular_velocity.y = angular_velocity[1];
  imu_data_.angular_velocity.z = angular_velocity[2];
  const double* linear_acc = imu_sensor_handle_.getLinearAcceleration();
  imu_data_.linear_acceleration.x = linear_acc[0];
  imu_data_.linear_acceleration.y = linear_acc[1];
  imu_data_.linear_acceleration.z = linear_acc[2];
  const double* orientation_cov = imu_sensor_handle_.getOrientationCovariance();
  imu_data_.orientation_covariance.assign(*orientation_cov);
  const double* angular_velocity_cov = imu_sensor_handle_.getAngularVelocityCovariance();
  imu_data_.angular_velocity_covariance.assign(*angular_velocity_cov);
  const double* linear_acc_cov = imu_sensor_handle_.getLinearAccelerationCovariance();
  imu_data_.linear_acceleration_covariance.assign(*linear_acc_cov);
  pub_.publish(imu_data_);
}
