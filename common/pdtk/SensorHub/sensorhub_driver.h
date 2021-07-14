#ifndef PedestrianTracker_SENSORHUB_DRIVER_H_
#define PedestrianTracker_SENSORHUB_DRIVER_H_

#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <unistd.h>
#include "utils/glogging.h"
#include "lidar_harvester.h"
#include "GlobalConstants.h"

namespace PedestrianTracker {

class SensorhubDriver {
public:
    SensorhubDriver();// {}
    virtual ~SensorhubDriver();// {}

    int Initialize();
    void Start();
    void Stop();
    void Destroy();
    void RegisterLidarCallback(LidarHarvester::callback_t cb);

protected:
    enum class Magic: uint8_t {
        kHead = 'T',
    };

  enum class DeviceType: uint8_t {
    kAccel    = 0x0A,
    kGyro     = 0x0B,
    kMag      = 0x0C,
    kBaro     = 0x0D,
    kBaroTemp = 0x0E,
    kCpuTemp  = 0x0F,
    kImuTemp  = 0x10,
    kLidar    = 0x11,
    kOdom     = 0x12,
    kMoto     = 0x13,
    kSonar    = 0x14,
    kIr       = 0x18,
    kBat      = 0x19,
    kTimer    = 0x1A,
    kDummy    = 0xFF,
  };

  struct ProtoHead {
    Magic magic;
    uint8_t type;
    DeviceType device;
    uint8_t length;
  } __attribute__ ((__packed__));

  struct ProtoImu {
    int16_t x;
    int16_t y;
    int16_t z;
  } __attribute__ ((__packed__));

  struct ProtoAcc {
    int16_t :4; int16_t x:12;
    int16_t :4; int16_t y:12;
    int16_t :4; int16_t z:12;
  } __attribute__ ((__packed__));

  struct ProtoLidar {
    uint16_t ts_increment;
    uint16_t angle_begin;
    uint16_t range[12];
    uint16_t angle_increment;
  } __attribute__ ((__packed__));

  struct ProtoOdom {
    int16_t left_speed;
    int16_t right_speed;
    int64_t left_mileage;
    int64_t right_mileage;
  } __attribute__ ((__packed__));

  struct ProtoIr {
    uint16_t range[5];
  } __attribute__ ((__packed__));

  struct ProtoSonar {
    uint16_t range[4];
  } __attribute__ ((__packed__));

  struct ProtoBody {
    uint64_t timestamp;
    union {
      ProtoAcc accel;
      ProtoImu gyro;
      ProtoImu mag;
      ProtoLidar lidar;
      ProtoOdom odom;
      ProtoIr ir;
      ProtoSonar sonar;
    };
  } __attribute__ ((__packed__));

  struct Proto {
    union {
      ProtoHead head;
      uint8_t head_buf[sizeof(ProtoHead)];
    };
    union {
      ProtoBody body;
      uint8_t body_buf[sizeof(ProtoBody)];
    };
    uint8_t crc8;
  } __attribute__ ((__packed__));

    typedef std::shared_ptr<Proto> ProtoPtr;
    typedef std::shared_ptr<const Proto> ProtoConstPtr;

 private:
    /* impl of SensorhubDevice::ControlType */
    int SetLidarPower(bool);

    int SetPower(int type, bool state);
    void OnProto(const Proto&);
    void ThreadLoop();

    virtual int RecvProto(Proto* proto) = 0;
    virtual int SendProto(const Proto* proto) = 0;

    std::unique_ptr<std::thread> thread_;
    std::unique_ptr<LidarHarvester> lidar_harvester_;
    bool running_ {false};
};

}

#endif
