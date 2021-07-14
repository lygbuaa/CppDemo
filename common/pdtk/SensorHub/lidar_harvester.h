#ifndef LIDAR_HARVESTER_H_
#define LIDAR_HARVESTER_H_

#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <tuple>
#include "utils/glogging.h"

namespace PedestrianTracker {

class LidarHarvester
{
public:
    typedef std::function<void (std::vector<std::tuple<float, float>>&& lidarPoints)> callback_t;

private:
  static constexpr int START_ANGLE_ = -80;
  static constexpr int STOP_ANGLE_ = 80;
  static constexpr int DEADLINE_ANGLE_ = 180;
  static constexpr double MAX_DELAY_S_ = 0.2f;
  static constexpr int FULL_SIZE_ = (STOP_ANGLE_ - START_ANGLE_) * 2 - 5; //allow 5 points missing

  double timestamp_ = 0.0f;
  int errorCnt_ = 0;
  std::vector<std::tuple<float, float>> lidarPoints_; //(deg, meter)

  callback_t fullCB_ = nullptr;

public:
  LidarHarvester(){
    lidarPoints_.clear();
  }
  ~LidarHarvester(){}

  void RegisterCallback(callback_t cb){
    fullCB_ = cb;
  }

  bool CollectPoint(const float angle, const float dist, const double ts){
    //save timestamp of the first point   
    if(lidarPoints_.empty()){
      timestamp_ = ts;
    }else if( ts - timestamp_ > MAX_DELAY_S_ ){
      lidarPoints_.clear();
      ++ errorCnt_;
      LOGW("exceed max delay: %f, total error: %d", ts - timestamp_, errorCnt_);
      return false;
    }else if( (angle > DEADLINE_ANGLE_) && !IsStorageFull() ){
      lidarPoints_.clear();
      ++ errorCnt_;
      LOGE("lidar angle reach deadline %d, but only %lu points collected, total error: %d", DEADLINE_ANGLE_, lidarPoints_.size(), errorCnt_);
      return false;
    }

    if(angle > START_ANGLE_ && angle < STOP_ANGLE_){
      lidarPoints_.emplace_back(std::make_tuple(angle, dist));
      LOGIO("collect lidar point: %.2f, %.2f, total: %d", angle, dist, lidarPoints_.size());

      if( IsStorageFull() ){
        /* need callback here */
        LOGIO("lidar points full: %d", lidarPoints_.size());
        if(fullCB_ != nullptr){
          fullCB_(std::move(lidarPoints_));
        }
        lidarPoints_.clear();
      }
    }
    return true;
  }

  bool IsStorageFull() const{
    return lidarPoints_.size() > FULL_SIZE_;
  }

};

}
#endif
