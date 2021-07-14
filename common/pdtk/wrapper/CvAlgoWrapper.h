//
// Created by hugoliu on 21-4-28.
//

#ifndef PEDESTRIANTRACKER_CVALGOWRAPPER_H
#define PEDESTRIANTRACKER_CVALGOWRAPPER_H

#include "inference_detection.h"
#include "inference_body_pose.h"
#include "inference_person_reid.h"
#include "kcf_tracker.h"
#include "GlobalConstants.h"
#include "utils/DepthUtils.h"
//#include <opencv2/tracking.hpp>
#include "LidarObserve.h"
#include "CameraObserve.h"
#include "UKFBodyTracker.h"
#include "Munkres.h"

namespace PedestrianTracker {

using person_t = PersonTarget3D_t<UKFBodyTracker>;
using Herds_t = std::unordered_map<uint32_t, person_t>;

class CvAlgoWrapper {
private:
    std::unique_ptr<InferenceDetection> pDetector_;
    std::unique_ptr<InferenceBodyPose> pBodyPose_;
    std::unique_ptr<InferenceReid> pReid_;
    std::unique_ptr<Munkres> pKM_;

    LidarCloud_t vLidarPoints_;
    Herds_t mHerds_;
    Herds_t localReidQueue_;
    uint32_t trackerCounter_ {0};
    uint32_t personCounter_ {0};

public:
    void LoadModels();
    std::vector<VisualTarget_t> Detect(const cv::Mat& raw, bool classify_pose = true);

    void MapLidar2Mono(const std::vector<std::tuple<float, float>>& lidarRawPoints);
    LidarCloud_t& GetLidarCloud();
    bool IsRoiValid(const cv::Rect& roi);
    bool IsPersonLonely(const person_t& person);
    bool IsFulcrumInside(const person_t& person);
    bool AddNewTracker(const cv::Mat& raw, const VisualTarget_t& vBody);
    void VisualAssignment(std::vector<VisualTarget_t>& newcomers);
    void LaserAssignment(const std::vector<LaserTarget_t>& newcomers);
    void ReidAssignment(person_t& newp);

    Herds_t GetHerds() const;
    void WashHerds();
    void UpdateStates(person_t& person);

    int InitHerds(const cv::Mat& raw);
    int TrackHerds(const cv::Mat& raw);

    float CompareTwoPerson(const person_t& old_person, const person_t& new_person);
    bool UpdateFeatures(const double now_t, const cv::Mat& raw, person_t& person);
    void AddNewFeature2Queue(const double now_t, person_t& person);

};
}
#endif //PEDESTRIANTRACKER_CVALGOWRAPPER_H
