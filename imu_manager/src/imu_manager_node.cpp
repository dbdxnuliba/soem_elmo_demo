#include <ros/ros.h>
#include <imu_manager/imu_manager.h>
#include <controller_manager/controller_manager.h>

std::string INTERFACE = "/dev/ttyS0";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_manager_node");
  ros::NodeHandle nh("~");

  std::string frame_id, interface, stop_service, reload_service, imu_topic, mode;
  int baudRate, rate;

  nh.getParam("interface", interface);
  nh.getParam("baudrate", baudRate);
  nh.getParam("rate", rate);
  nh.getParam("mode", mode);

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("frame_id",frame_id);
  nh.getParam("reload_service_name", reload_service);
  nh.getParam("stop_service_name", stop_service);

  ImuManager imu(nh, stop_service, reload_service, imu_topic,
                 baudRate, rate, frame_id, interface);
  //controller_manager::ControllerManager cm(&imu, nh);
  ros::Rate rate_t(1.0 / imu.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){
    //imu.read();
    //cm.update(imu.getTime(), imu.getPeriod());
    //imu.write();
    rate_t.sleep();
  }
  spinner.stop();
  return 0;
}
