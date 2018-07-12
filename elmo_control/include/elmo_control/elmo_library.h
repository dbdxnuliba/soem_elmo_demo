#ifndef ELMO_LIBRARY_H
#define ELMO_LIBRARY_H

#include <elmo_control/elmo_client.h>
#include <ethercat_manager/ethercat_manager.h>
#include <vector>

namespace elmo_control {
class ElmoClient;
}


namespace elmo {

typedef struct{
  double vel_feedback_;
  double pos_feedback_;
  double effort_feedback_;
}elmo_feedback;

class ElmoLibrary{
public:
  ElmoLibrary(const std::string& ifname);
  void initialize();
  std::vector<elmo_feedback> readDataFromElmo();
  void writeToElmo(std::vector<double>& velocities);
private:
  //ethercat::EtherCatManager manager_;

  std::string ifname_;
  std::vector<elmo_feedback> feeds_;
  std::vector<elmo_control::ElmoClient *> clients_;
  int slave_no_;

};

} //end namespace

#endif // ELMO_LIBRARY_H
