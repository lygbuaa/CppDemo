#ifndef PedestrianTracker_UART_SENSORHUB_DRIVER_H_
#define PedestrianTracker_UART_SENSORHUB_DRIVER_H_

#include "sensorhub_driver.h"
#include <string>
#include <memory>
#include <sys/select.h>

namespace PedestrianTracker {

class UartSensorhubDriver: public SensorhubDriver {
 public:
  UartSensorhubDriver();// {}
  virtual ~UartSensorhubDriver();// {}

  int Initialize();
  void Start();
  void Stop();
  void Destroy();

 private:
  enum class State {
    kMagic,
    kType,
    kDevice,
    kLength,
    kTimestame,
    kBody,
    kCrc,
  };

  int OpenTTY();
  int RestartTTY();
  int ReadTTY(uint8_t* buf, int size, int fd, struct timeval *timeout = NULL);
  int RecvProto(Proto* proto) override;
  int SendProto(const Proto* proto) override;

  void Crc8Init();
  uint8_t Crc8(const uint8_t* buf, int n);

  int fd_ {-1};
  uint8_t buf_[1024];
  uint8_t* buf_head_ {buf_};
  uint8_t* buf_tail_ {0};
  State state_ {State::kMagic};
  int proto_body_index_ {0};
  Proto proto_;

  uint8_t crc8_table_[256];

  const std::string dev_name_ {GlobalConstants::TTY_FILE_}; //"/dev/ttyACM0"
  const std::string name_ {"UartSensorhub"};
};

}

#endif