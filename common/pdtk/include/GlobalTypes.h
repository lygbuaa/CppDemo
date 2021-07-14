//
// Created by hugoliu on 21-5-21.
//

#ifndef PEDESTRIANTRACKER_GLOBALTYPES_H
#define PEDESTRIANTRACKER_GLOBALTYPES_H

#include "base_infer_def.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace PedestrianTracker {
    enum class WorkingMode_t : int {
        SINGLE_TRACKING = 2,
        MULTI_TRACKING = 3,
    };

    enum class SenseDomain_t : int {
        LIDAR = 0,
        VISUAL = 1,
    };

    struct Interval_t{
        SenseDomain_t domain {SenseDomain_t::LIDAR};
        float startAngle {0.0f};
        float endAngle {0.0f};
        int startPixel {0};
        int endPixel {0};
    };

//    using BodyTemplates_t = std::unordered_map<int, std::vector<float>>;
    struct VisualTarget_t{
        cv::Rect roi;
        std::unordered_map<int, std::vector<float>> vFeats;
        double feats_tick_s {-1.0f};
        BODY_POSE orientation {BODY_POSE_UNCERTAIN};
        float det_score {0.0f};
        std::vector<std::pair<uint32_t, double>> iou_scores;
        std::vector<std::pair<uint32_t, double>> reid_scores;

        void update(const VisualTarget_t& t){
            roi = t.roi;
            orientation = t.orientation;
            det_score = t.det_score;
        }
    };

    struct LaserTarget_t{
        int startIdx {-1};
        float start_theta {0.0f};
        float start_dist {0.0f};
        int endIdx {-1};
        float end_theta {0.0f};
        float end_dist {0.0f};
        float width {0.0f};

        int count {0};
        float sum_dist {0.0f};
        float sum_theta {0.0f};
        float center_dist {-1.0f};
        float center_theta {0.0f};
        std::vector<std::pair<uint32_t, double>> dist_scores;

        void init(int idx, float dist, float theta){
            startIdx = idx;
            start_theta = theta;
            start_dist = dist;
            endIdx = idx;
            end_theta = theta;
            end_dist = dist;

            count = 1;
            sum_dist = dist;
            sum_theta = theta;
            center_dist = dist;
            center_theta = theta;
        }

        void expand(int idx, float dist, float theta){
            endIdx = idx;
            end_theta = theta;
            end_dist = dist;

            count += 1;
            sum_dist += dist;
            sum_theta += theta;
            center_dist = sum_dist / count;
            center_theta = sum_theta / count;
        }

        void close(int idx){
            double diff_rad = (end_theta - start_theta) / 57.3f;
            width = sqrt(start_dist*start_dist + end_dist*end_dist - 2*start_dist*end_dist*cos(diff_rad));
        }

        LaserTarget_t& merge(const LaserTarget_t& t){
            if(t.startIdx < this->startIdx){
                this->startIdx = t.startIdx;
                this->start_dist = t.start_dist;
                this->start_theta = t.start_theta;
            }
            if(t.endIdx > this->endIdx){
                this->endIdx = t.endIdx;
                this->end_dist = t.end_dist;
                this->end_theta = t.end_theta;
            }
            double diff_rad = (end_theta - start_theta) / 57.3f;
            this->width = sqrt(start_dist*start_dist + end_dist*end_dist - 2*start_dist*end_dist*cos(diff_rad));

            this->count += t.count;
            this->sum_dist += t.sum_dist;
            this->sum_theta += t.sum_theta;
            this->center_dist = this->sum_dist / this->count;
            this->center_theta = this->sum_theta / this->count;
            return *this;
        }
    };

    static bool LaserTargetComparison(LaserTarget_t& first, LaserTarget_t& second){
        return (first.center_dist < second.center_dist);
    }

    struct LidarInMono_t{
        int x {-1};
        int y {-1};
        float z {-1.0f};
        bool invisible {false};
    };

    struct LidarRawPoint_t{
        float dist {-1.0f};
        float deg {0.0f};
        float rad {0.0f};
    };

    struct LidarPoint_t{
        LidarRawPoint_t raw;
        LidarInMono_t inMono;
    };

    using LidarCloud_t = std::vector<LidarPoint_t>;

    static bool LidarPointComparison(LidarPoint_t& first, LidarPoint_t& second){
        return (first.raw.deg < second.raw.deg);
    }

    template <typename KalmanType>
    struct PersonTarget3D_t{
        /* unique id of any person, only increase */
        uint32_t uid {0};
        /* name the person after reid_score */
        int32_t rid {-1};
        /* the matched visual target */
        VisualTarget_t vTarget;
        /* the matched laser target */
        LaserTarget_t lTarget;
        /* legs center point in MONO */
        cv::Point2d fulcrum;
        /* UKFBodyTracker pointer */
        std::shared_ptr<KalmanType> pKf {nullptr};
        std::vector<double> states;
        /* delete this tracker, if disappear more than 5 frame */
        uint32_t visual_disappear {0};
        uint32_t laser_disappear {0};
        /* indicator for ukf observe */
        bool visual_match {false};
        bool laser_match {false};
    };
}

#endif //PEDESTRIANTRACKER_GLOBALTYPES_H
