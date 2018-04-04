#ifndef ETHERCAT_MANAGER_H
#define ETHERCAT_MANAGER_H

#include <stdexcept>
#include <string>
#include <stdint.h>

#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


namespace ethercat {


class EtherCatError : public std::runtime_error{
public:
  explicit EtherCatError(const std::string& what): std::runtime_error(what){}
};


class EtherCatManager{
public:
  EtherCatManager(const std::string& ifname);
  ~EtherCatManager();

  void write(int slave_no, uint8_t channel, uint8_t value);
  uint8_t readInput(int slave_no, uint8_t channel) const;
  uint8_t readOutput(int slave_no, uint8_t channel) const;

  template<typename T>
  uint8_t writeSDO(int slave_no, uint8_t index, uint8_t subidx, T value) const;

  template<typename T>
  T readSDO(int slave_no, uint8_t index, uint8_t subidx, T value) const;

  int getNumClients() const;

  void getStatus(int slave_no, std::string &name, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state,
                 int &pdelay, int &hasdc, int &activeports, int &configadr) const;

private:
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
