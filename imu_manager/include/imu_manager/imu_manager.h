#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>


#include "vn/sensors.h"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

/**
 * @brief Class for Interfacing with VectorNav imu inheriting from
 * hardware_interface::RobotHw
 */
class ImuManager : public hardware_interface::RobotHW{

  /**
   * @brief Constructor
   * @param node Nodehandle
   * @param stop_service name of the stopping service
   * @param reload_service name of the reloading service
   * @param imu_topic name of topic where data will be published
   * @param baudRate controller frequency
   * @param timeOut timeout in mSec
   * @param frame_id the frame where data will be published
   */
  ImuManager(ros::NodeHandle &node, const std::string& stop_service,
             const std::string& reload_service, const std::string imu_topic,
             int baudRate, int timeOut, const std::string frame_id);


  /**
   * @brief function to read the sensor data
   */
  void read();

  /**
   * @brief function to publish sensor data
   */
  void write();

  /**
   * @brief callback function to answer reload service
   * @return
   */
  bool reload_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * @brief callback function to answer stop service
   * @return
   */
  bool stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);







private:
  ros::NodeHandle nh_;
  ros::ServiceServer stop_service_;
  ros::ServiceServer reload_service_;
  bool running_;
  ros::Publisher pub_;
  std::string frame_id_;

  int baudRate_;
  int timeOut_;

  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;

  /**
   * @brief function to return time
   * @return
   */
  ros::Time getTime() const;

  /**
   * @brief function to return period
   * @return
   */
  ros::Duration getPeriod() const;


};




#endif // IMU_MANAGER_H
