//
// Created by hugoliu on 21-5-21.
//

#include "LidarObserve.h"

namespace PedestrianTracker {

    std::vector<LaserTarget_t> LidarObserve::ClusterLidarInterval(const LidarCloud_t& cloud, const Interval_t& interval){
        using namespace GlobalConstants;

        std::vector<LaserTarget_t> vTargets;
        LaserTarget_t target;
        target.center_dist = -1.0f;
        target.count = 0;
        for(int i = 0; i < cloud.size(); ++i){
            const float& angle_deg = cloud[i].raw.deg;
            const float& dist_m = cloud[i].raw.dist;
            if(dist_m < 0.05f){
                continue;
            }
            const int& pixel = cloud[i].inMono.x;

            /* border condition */
            if(interval.domain == SenseDomain_t::LIDAR) {
                if (angle_deg < interval.startAngle) {
                    continue;
                } else if (angle_deg > interval.endAngle) {
                    if (target.count >= LIDAR_CLUSTER_MIN_COUNT_) {
                        target.close(i);
                        vTargets.emplace_back(target);
                    }
                    break;
                }
            }else if(interval.domain == SenseDomain_t::VISUAL){
                if(pixel < interval.startPixel){
                    continue;
                }else if(pixel > interval.endPixel){
                    if(target.count >= LIDAR_CLUSTER_MIN_COUNT_){
                        target.close(i);
                        vTargets.emplace_back(target);
                    }
                    break;
                }
            }else{
                break;
            }

            const float diff = fabs(dist_m - target.center_dist);
            if(diff > LIDAR_CLUSTER_MIN_DIFF_M_){
                /* save last target if valid */
                if(target.count >= LIDAR_CLUSTER_MIN_COUNT_){
                    target.close(i);
                    vTargets.emplace_back(target);
                }
                /* new target */
                target.init(i, dist_m, angle_deg);
            }else{
                target.expand(i, dist_m, angle_deg);
            }
        }
        return vTargets;
    }

    bool LidarObserve::IsLaserTargetValid(const LaserTarget_t& target){
        return (target.width > GlobalConstants::LIDAR_TARGET_MIN_WIDTH_) && (target.width < GlobalConstants::LIDAR_TARGET_MAX_WIDTH_);
    }

    void LidarObserve::WashLaserTargets(std::vector<LaserTarget_t>& old_targets){
        std::vector<int> trash;
        for(int i = 0; i < old_targets.size(); ++i){
            if( !IsLaserTargetValid(old_targets[i]) ){
                trash.emplace_back(i);
            }
        }
        std::sort(trash.begin(), trash.end());
        while( !trash.empty() ){
            size_t idx = trash.back();
//            LOGI("erase invalid target (%.2f, %.2f, %.2f)", old_targets[idx].center_theta, old_targets[idx].center_dist, old_targets[idx].width);
            old_targets.erase(old_targets.begin() + idx);
            trash.pop_back();
        }
    }

    int LidarObserve::PickLegsFromInterval(const LidarCloud_t& cloud, const Interval_t& interval, LaserTarget_t& legs){
        std::vector<LaserTarget_t> targets = ClusterLidarInterval(cloud, interval);
        if(targets.empty()){
            return -1;
        }
        WashLaserTargets(targets);
        std::sort(targets.begin(), targets.end(), LaserTargetComparison);

        legs = targets[0];
        if(targets.size() == 1){
            return 1;
        }else if(TargetsNear(targets[0], targets[1])){
            /* second target near enough */
            legs.merge(targets[1]);
            return 2;
        }else{
            /* second target faraway */
            return 3;
        }
    }

    bool LidarObserve::RoiFloatingOnLaser(const LidarCloud_t& cloud, const cv::Rect& roi){
        const int startPixel = roi.x;
        const int stopPixel = roi.x + roi.width;
        LidarCloud_t points_in_roi;
        for(auto point : cloud){
            if( point.inMono.x > startPixel && point.inMono.x < stopPixel && point.inMono.y > roi.y && point.inMono.y < (roi.y+roi.height)){
                points_in_roi.emplace_back(point);
            }
        }
        return points_in_roi.empty();
    }

    /* theta: degree;  dist: meter */
    double LidarObserve::PolarDistance(const double theta1, const double dist1, const double theta2, const double dist2){
        double diff_rad = (theta1 - theta2) / 57.3f;
        return sqrt(dist1*dist1 + dist2*dist2 - 2*dist1*dist2*cos(diff_rad));
    }

    bool LidarObserve::TargetsNear(const LaserTarget_t& target1, const LaserTarget_t& target2){
        double dist = PolarDistance(target1.center_theta, target1.center_dist, target2.center_theta, target2.center_dist);
        return (dist < GlobalConstants::LIDAR_TARGETS_NEAR_M_);
    }

    Interval_t LidarObserve::GetTrackerInterval(const double theta, const double dist, const double width){
        double alpha = atan(width/2.0f / dist) * 57.3 * GlobalConstants::LIDAR_INTERVAL_INFLATE_;
        Interval_t interval;
        interval.startAngle = theta - alpha;
        interval.endAngle = theta + alpha;
        return interval;
    }

    bool LidarObserve::TargetsFusionable(const LaserTarget_t& target1, const LaserTarget_t& target2){
        bool connected = (abs(target1.startIdx - target2.startIdx) < 4)
                      || (abs(target1.endIdx - target2.endIdx) < 4)
                      || (abs(target1.startIdx - target2.endIdx) < 4)
                      || (abs(target2.startIdx - target1.endIdx) < 4);
        const double dist = PolarDistance(target1.center_theta, target1.center_dist, target2.center_theta, target2.center_dist);
        return connected && (dist < GlobalConstants::LIDAR_CLUSTER_MIN_DIFF_M_);
    }

    bool LidarObserve::TryFuseNewTarget(std::vector<LaserTarget_t>& old_targets, const LaserTarget_t& new_target){
        for(auto& old_t : old_targets){
            if(TargetsFusionable(old_t, new_target)){
                old_t.merge(new_target);
                LOGI("new target [%.2f, %.2f, %.2f] fused to [%.2f, %.2f, %.2f]", new_target.center_theta, new_target.center_dist, new_target.width, old_t.center_theta, old_t.center_dist, old_t.width);
                return true;
            }
        }
        return false;
    }

}