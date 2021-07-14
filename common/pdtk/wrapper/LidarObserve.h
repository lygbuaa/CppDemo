//
// Created by hugoliu on 21-5-17.
//

#ifndef PEDESTRIANTRACKER_LIDAROBSERVE_H
#define PEDESTRIANTRACKER_LIDAROBSERVE_H

#include <Eigen/Eigen>
#include "GlobalConstants.h"
#include "GlobalTypes.h"
#include "utils/DepthUtils.h"

namespace PedestrianTracker {
class LidarObserve {
private:

public:
    explicit LidarObserve(){
    }
    LidarObserve(const LidarObserve&) = delete;
    ~LidarObserve(){}

    static std::vector<LaserTarget_t> ClusterLidarInterval(const LidarCloud_t& cloud, const Interval_t& interval);
    static bool IsLaserTargetValid(const LaserTarget_t& target);
    static void WashLaserTargets(std::vector<LaserTarget_t>& old_targets);

    /* use this sugar only on initial */
    static int PickLegsFromInterval(const LidarCloud_t& cloud, const Interval_t& interval, LaserTarget_t& legs);
    static bool RoiFloatingOnLaser(const LidarCloud_t& cloud, const cv::Rect& roi);
    static double PolarDistance(const double theta1, const double dist1, const double theta2, const double dist2);
    static bool TargetsNear(const LaserTarget_t& target1, const LaserTarget_t& target2);
    static Interval_t GetTrackerInterval(const double theta, const double dist, const double width);
    static bool TargetsFusionable(const LaserTarget_t& target1, const LaserTarget_t& target2);
    static bool TryFuseNewTarget(std::vector<LaserTarget_t>& old_targets, const LaserTarget_t& new_target);

};
}

#endif //PEDESTRIANTRACKER_LIDAROBSERVE_H
