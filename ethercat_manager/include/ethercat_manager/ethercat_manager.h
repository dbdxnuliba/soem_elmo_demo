/**
 *  (c) 2018, Abhijit Makhal.
 *  Ethercat Manager
 *  Reading data from driver and writing data to driver (ELMO) using SOEM library
 */

#ifndef ETHERCAT_MANAGER_H
#define ETHERCAT_MANAGER_H

#include <stdexcept>
#include <string>
#include <stdint.h>

#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


namespace ethercat {
/**
 * @brief The EtherCatError exception. This is thrown when there
 * is a failure in constructing an ethercat manager
 */

class EtherCatError : public std::runtime_error{
public:
  explicit EtherCatError(const std::string& what): std::runtime_error(what){}
};

/**
 * @brief The EtherCatManager class
 * This class provides a CPP interface to the SOEM library
 * Provided the name of the ethernet device, e.g. "eth0", it will connect,
 * start a thread that cycles data around the network, and provive
 * read/write access to the underlying IO map
 */
class EtherCatManager{
public:
  /**
   * @brief EtherCatManager Constructs and initializes the ethercat slaves on a given network
   * @param ifname The name of the netwrk interface the ethercat chain is connected to [eth0]
   */

  EtherCatManager(const std::string& ifname);

  ~EtherCatManager();

  /**
   * @brief writes value to this channel's output register of the given slave
   * @param slave_no The slave number of the device (starts from 1)
   * @param channel The byte offset into the output IOMap to write values to
   * @param value The byte value to write
   */
  void write(int slave_no, uint8_t channel, uint8_t value);

  /**
   * @brief reads this chennel's input-register of the given name
   * @param slave_no The slave number of the device (starts from 1)
   * @param channel The byte offset into the input IOMap to read from
   * @return
   */
  uint8_t readInput(int slave_no, uint8_t channel) const;

  /**
   * @brief reads this chennel's output-register of the given name
   * @param slave_no The slave number of the device (starts from 1)
   * @param channel The byte offset into the input IOMap to read from
   * @return
   */
  uint8_t readOutput(int slave_no, uint8_t channel) const;


  /**
   * @brief writes the SDO object to the given slave number
   * @param slave_no The slave number of the device (starts from 1)
   * @param index The index address of the parameter in the SDO object
   * @param subidx The sub-index addess of the parameter in the SDO object
   * @param value The byte offset into the input IOMap to read from
   * @return
   */
  template<typename T>
  uint8_t writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const;


  /**
   * @brief readss the SDO object to the given slave number
   * @param slave_no The slave number of the device (starts from 1)
   * @param index The index address of the parameter in the SDO object
   * @param subidx The sub-index addess of the parameter in the SDO object
   * @return
   */
  template<typename T>
  T readSDO(int slave_no, uint16_t index, uint8_t subidx) const;

  /**
   * @brief returns the number of clients
   * @return int
   */
  int getNumClients() const;

  /**
   * @brief Obtain the status of the client
   * @param slave_no The slave number of the device (starts from 1)
   * @param name This slave's name
   * @param eep_man This slave's manufacturer
   * @param eep_id This slave's manufacturer id
   * @param eep_rev This slave's revision
   * @param obits OutputBits
   * @param ibits InputBits
   * @param state Current state
   * @param pdelay Pdelay
   * @param hasdc hasDC
   * @param activeports activeports
   * @param configadr configaddress
   */
  void getStatus(int slave_no, std::string &name, int &eep_man, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state,
                 int &pdelay, int &hasdc, int &activeports, int &configadr) const;

private:
  /**
   * @brief Intialize SOEM library
   * @param ifname The name of the netwrk interface the ethercat chain is connected to [eth0]
   * @return
   */
  bool initSoem(const std::string& ifname);

  const std::string ifname_;
  uint8_t iomap_[4096];
  int num_clients_;
  boost::thread cycle_thread_;
  mutable boost::mutex iomap_mutex_;
  bool stop_flag_;
};

}

#endif // ETHERCAT_MANAGER_H
