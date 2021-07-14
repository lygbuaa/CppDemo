#include "uart_sensorhub_driver.h"
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

namespace PedestrianTracker {

UartSensorhubDriver::UartSensorhubDriver() {}
UartSensorhubDriver::~UartSensorhubDriver() {}

int UartSensorhubDriver::OpenTTY() {
  int flag = 0;
  flag = O_RDWR | O_NOCTTY;
  fd_ = open(dev_name_.c_str(), flag);
  LOGI("open serial %s fd: %d, errno: %s", dev_name_.c_str(), fd_, strerror(errno));
//  LOG(ERROR) << "open serial " << dev_name_.c_str() << " return: " << fd_ << ", errno: " << errno << "(" << strerror(errno) << ")";
  CHECK(fd_ >= 0);

  struct termios opt;
  int ret;
  ret = tcgetattr(fd_, &opt);
  CHECK(ret == 0);

  opt.c_cflag = B921600 | CRTSCTS | CS8 | CLOCAL | CREAD;
  opt.c_iflag = IGNPAR;
  opt.c_oflag = 0;
  opt.c_lflag = 0; //ICANON;
  opt.c_cc[VTIME] = 1;
  opt.c_cc[VMIN] = 255;
  tcflush(fd_, TCIFLUSH);
  ret = tcsetattr(fd_, TCSANOW, &opt);
  CHECK(ret == 0);
  Crc8Init();
  return ret;
}

int UartSensorhubDriver::Initialize() {
  LOGI("Initialize");
  OpenTTY();
  return SensorhubDriver::Initialize();
}

void UartSensorhubDriver::Start() {
  LOGI("Start");
  SensorhubDriver::Start();
}

void UartSensorhubDriver::Stop() {
  LOGI("Stop");
  if (fd_ >= 0) {
    close(fd_);
  }
  SensorhubDriver::Stop();
}

void UartSensorhubDriver::Destroy() {
  LOGI("Destroy");
  SensorhubDriver::Destroy();
  if (fd_ >= 0) {
    close(fd_);
  }
}

int UartSensorhubDriver::RestartTTY(){
  LOGW("close & re-open tty");
  if (fd_ >= 0) {
    close(fd_);
  }
  return OpenTTY();;
}

int UartSensorhubDriver::ReadTTY(uint8_t* buf, int size, int fd, struct timeval *timeout){
  fd_set fdset;
  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);

  int ret = select(fd + 1, &fdset, NULL, NULL, timeout);
  if (ret == 0) {
    LOGW("fd %d select timeout", fd);
    /* re-open tty */
    return RestartTTY();
  } else if(ret < 0) {
    LOGW("fd %d select error: %s", fd, strerror(errno));
    return ret;
  }

  if(FD_ISSET(fd, &fdset)){
    ret = read(fd, buf, size);
    if(ret == 0) {
      LOGE("reach tty end, please retry later!");
    } else if (ret < 0){
      LOGE("read tty error: %s", strerror(errno));
    } else {
      LOGIO("read %d bytes from tty", ret);
    }
    FD_CLR(fd, &fdset);
  }
  return ret;
}

int UartSensorhubDriver::RecvProto(Proto* proto) {
  if (buf_head_ > buf_tail_) {
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100*1000;
    int n_read = 0;
    do{
      n_read = ReadTTY(buf_, sizeof(buf_), fd_, &timeout);
      if (n_read < 0) {
        return -1;
      } else if (n_read == 0) {
        usleep(10*1000);
      } else {
        /* data valid */
        break;
      }
    }while(true);

    buf_head_ = buf_;
    buf_tail_ = buf_ + n_read - 1;
  }
  bool is_proto_valid = false;
  while (buf_head_ <= buf_tail_) {
    switch (state_) {
      case State::kMagic:
        if (*buf_head_ != static_cast<uint8_t>(Magic::kHead)) {
          //LOG(INFO) << "not magic :" << static_cast<int>(*buf_head_);
          buf_head_++;
        } else {
          proto_.head.magic = Magic::kHead;
          state_ = State::kType;
          buf_head_++;
          //LOG(INFO) << "magic";
        }
        break;
      case State::kType:
        proto_.head.type = *buf_head_;
        state_ = State::kDevice;
        buf_head_++;
        //LOG(INFO) << "type: " << static_cast<int>(proto_.head.type);
        break;
      case State::kDevice:
        proto_.head.device = DeviceType(*buf_head_);
        state_ = State::kLength;
        buf_head_++;
        //LOG(INFO) << "device: " << static_cast<int>(proto_.head.device);
        break;
      case State::kLength:
        proto_.head.length = *buf_head_;
        state_ = State::kBody;
        buf_head_++;
        //LOG(INFO) << "length: " << static_cast<int>(proto_.head.length);
        break;
      case State::kBody: {
        int length = std::min((int)(buf_tail_ - buf_head_ + 1),
                              (int)proto_.head.length - proto_body_index_);
        //LOG(INFO) << "to copy: " << length;
        std::memcpy(proto_.body_buf + proto_body_index_, buf_head_, length);
        //printf("raw \n");
        //for (int i = 0; i < length; i++) {
        //  printf("0x%02X ", buf_head_[i]);
        //}
        //printf("\n");
        proto_body_index_ += length;
        if (proto_body_index_ < proto_.head.length) {
          buf_head_ += length;
        } else {
          state_ = State::kCrc;
          proto_body_index_ = 0;
          buf_head_ += length;
        }
        break;
      }
      case State::kCrc:
        proto_.crc8 = *buf_head_;
        state_ = State::kMagic;
        uint8_t crc8_calc = Crc8(reinterpret_cast<uint8_t*>(&proto_),
                                 proto_.head.length + sizeof(ProtoHead));
        if (proto_.crc8 == crc8_calc) {
          is_proto_valid = true;
        } else {
          is_proto_valid = false;
//          LOG(ERROR) << "crc8 error";
            LOGE("crc8 error");
        }
        buf_head_++;
        //LOG(INFO) << "crc8 recv: " << static_cast<int>(proto_.crc8);
        //LOG(INFO) << "crc8 calc: " << static_cast<int>(crc8_calc);
        break;
    }
    if (is_proto_valid) {
      break;
    }
  }
  if (is_proto_valid) {
    *proto = proto_;
    return 0;
  } else {
    return 1;
  }
}

int UartSensorhubDriver::SendProto(const Proto* proto) {
  return 0;
}

void UartSensorhubDriver::Crc8Init() {
  int i, j;
  const uint8_t msbit = 0x80;
  uint8_t t = msbit;
  uint8_t polynomial = 0x4d;

  crc8_table_[0] = 0;

  for (i = 1; i < sizeof(crc8_table_); i *= 2) {
    t = (t << 1) ^ (t & msbit ? polynomial : 0);
    for (j = 0; j < i; j++)
      crc8_table_[i+j] = crc8_table_[j] ^ t;
  }
}

uint8_t UartSensorhubDriver::Crc8(const uint8_t* buf, int n) {
  uint8_t crc8 = 0xFF;
  while (n-- > 0) {
    crc8 = crc8_table_[(crc8 ^ *buf++) & 0xff];
  }
  return 0xFF ^ crc8;
}

}