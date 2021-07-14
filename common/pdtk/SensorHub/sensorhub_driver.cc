#include "sensorhub_driver.h"

namespace {
  constexpr float kLidarAngleFsr = 0.01 * 3.1416f / 180.0;
  constexpr float kLidarRangeFsr = 0.01; // 0.01m
  constexpr float kAccelFsr = 0.009581; // accel_fsr in HAL sensors
  constexpr float kGyroFsr = 0.000266; // gyro_fsr in HAL sensors
  constexpr float kIrFsr = 0.001; // 0.001m
  constexpr float kMeterPerStep = 0.000107;
  constexpr double kUSecondsToSeconds = 1e-6;
}

namespace PedestrianTracker {

SensorhubDriver::SensorhubDriver(){}
SensorhubDriver::~SensorhubDriver(){}

void SensorhubDriver::RegisterLidarCallback(LidarHarvester::callback_t cb){
    if(lidar_harvester_){
        lidar_harvester_ -> RegisterCallback(cb);
    }
}

int SensorhubDriver::Initialize() {
    lidar_harvester_.reset(new LidarHarvester());
    return 0;
}

void SensorhubDriver::Start() {
    if (running_) {
        return;
    }
    running_ = true;
    thread_.reset(new std::thread(&SensorhubDriver::ThreadLoop, this));
}

void SensorhubDriver::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    thread_->join();
}

void SensorhubDriver::Destroy() {
}

/* impl of SensorhubDevice::ControlType */
int SensorhubDriver::SetLidarPower(bool) {
    return 0;
}

int SensorhubDriver::SetPower(int type, bool state) {
    return 0;
}

void SensorhubDriver::OnProto(const Proto& proto) {
    switch (proto.head.device) {
        case DeviceType::kAccel: {
//            LOGI("kAccel: %d", proto.head.length);
            break;
        }
        case DeviceType::kGyro: {
//            LOGI("kGyro: %d", proto.head.length);
            break;
        }
        case DeviceType::kLidar: {
            int normal_count = 0;
            for (auto& range: proto.body.lidar.range) {
                if (range == 0xFFFF) {
                    break;
                }
                normal_count++;
            }
            LOGIO("kLidar total: %d, normal_count: %d", proto.head.length, normal_count);

            if (normal_count == 0) {
                break;
            }
            uint64_t ts_end = proto.body.timestamp;
            uint16_t ts_increment = proto.body.lidar.ts_increment;
            uint64_t ts_begin = ts_end - ts_increment * (normal_count - 1);
//            float angle_begin = proto.body.lidar.angle_begin * kLidarAngleFsr;
//            float angle_increment = proto.body.lidar.angle_increment * kLidarAngleFsr;

            for (int i = 0; i < normal_count; i++) {
//                float angle = 2 * 3.1416f - (angle_begin + angle_increment * i);
                float range = proto.body.lidar.range[i] * kLidarRangeFsr;
                double timestamp = (ts_begin + ts_increment * i) * kUSecondsToSeconds;
                //angle in degree
                float fov_angle = (proto.body.lidar.angle_begin + proto.body.lidar.angle_increment * i) * 0.01;
                //map into -180 ~ +180
                if(GlobalConstants::MINI_LIDAR_TRANSPOSE_){
                    fov_angle -= 180.0f;
                }else{
                    if(fov_angle > 180.0f){
                        fov_angle -= 360.0f;
                    }
                }
                lidar_harvester_ -> CollectPoint(fov_angle, range, timestamp);
            }
            break;
        }
        case DeviceType::kOdom: {
//            LOGI("kOdom: %d", proto.head.length);
            break;
        }
        case DeviceType::kIr: {
//            LOGI("kIr: %d", proto.head.length);
            break;
        }
        case DeviceType::kTimer: {
            double timestamp = proto.body.timestamp * kUSecondsToSeconds;
//            LOGI("kTimer: %d", proto.head.length);
            break;
        }
        default:
            break;
    }
}

void SensorhubDriver::ThreadLoop() {
    Proto proto;
    LOGI("start sensorhub RecvProto loop");
    while (running_) {
        int ret = RecvProto(&proto);
        if (ret < 0) {
            LOGE("sensorhub RecvProto error: %d", ret);
            break;
        } else if (ret > 0) {
            // LOG(ERROR) << "[lxc] sensorhub RecvProto invalid: " << ret;
            continue;
        } else {
            OnProto(proto);
        }
    }
    LOGI("sensorhub RecvProto loop exit");
}

}