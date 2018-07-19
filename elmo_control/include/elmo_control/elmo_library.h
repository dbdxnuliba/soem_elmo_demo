#ifndef ELMO_LIBRARY_H
#define ELMO_LIBRARY_H

#include <elmo_control/elmo_client.h>
#include <ethercat_manager/ethercat_manager.h>
#include <vector>

namespace elmo_control {
class ElmoClient;
}


namespace elmo {

/**
  * @brief Structure for feedback from Elmo drives
  *
  */
typedef struct{
  double vel_feedback;
  double pos_feedback;
  double effort_feedback;
}elmo_feedback;

/**
 * @brief The ElmoLibrary class
 * Simple high level wrapper to access the elmo drives
 */
class ElmoLibrary{
public:
  /**
   * @brief ElmoLibrary Constructor creates the manager and sets the clients
   * @param ifname the name of the interface
   */
  ElmoLibrary(const std::string& ifname);

  ~ElmoLibrary();

  /**
   * @brief initialize initializes the drives. resets everything
   */
  void initialize();

  /**
   * @brief readDataFromElmo reads data from elmo drive
   * @return vector of feedback struct for every drive
   */
  std::vector<elmo_feedback> readDataFromElmo();

  /**
   * @brief writeToElmo writes to elmo drives
   * @param velocities vector of velocities
   */
  void writeToElmo(std::vector<double>& velocities);

  /**
   * @brief getNumberofClients number of clients available
   * useful for setting the number of velocities
   * @return int number of clinets
   */
  int getNumberofClients();
private:
  ethercat::EtherCatManager* manager_;
  std::string ifname_;
  std::vector<elmo_control::ElmoClient *> clients_;
  int slave_no_;



};

} //end namespace

#endif // ELMO_LIBRARY_H
