//
// Created by hugoliu on 21-4-28.
//

#include "CvAlgoWrapper.h"

namespace PedestrianTracker {

void CvAlgoWrapper::LoadModels() {
    using namespace GlobalConstants;
    std::string model_path(ASSETS_PATH_);
    CameraObserve::InitParams();

    std::string model_detector = model_path + CV_DETECTION_MODEL_;
    pDetector_ = std::unique_ptr<InferenceDetection> (new InferenceDetection(model_detector.c_str()));

    std::string model_bodypose = model_path + CV_BODYPOSE_MODEL_;
    pBodyPose_ = std::unique_ptr<InferenceBodyPose> (new InferenceBodyPose(model_bodypose.c_str()));

    std::string model_bodyreid = model_path + CV_BODYREID_MODEL_;
    pReid_ = std::unique_ptr<InferenceReid> (new InferenceReid(model_bodyreid.c_str()));

    pKM_ = std::unique_ptr<Munkres>(new Munkres());

}

std::vector<VisualTarget_t> CvAlgoWrapper::Detect(const cv::Mat& raw, bool classify_pose){
    std::vector<BBox> bboxes = pDetector_ -> detect(raw);
    std::vector<VisualTarget_t> bodies;
    for(auto b : bboxes){
        VisualTarget_t body;
        body.roi = bbox2rect(b);
        body.det_score = b.score;
        cv::Mat gray = raw(body.roi);
        if(classify_pose){
            HANG_STOPWATCH();
            std::vector<float> probs;
            cv::Mat body_bgr;
            if(gray.channels() == 1){
                cv::cvtColor(gray, body_bgr, cv::COLOR_GRAY2BGR);
            }
            body.orientation = (BODY_POSE)(pBodyPose_->classify(body_bgr, probs));
        }

//        float body_w = 0.0f;
//        float body_h = 0.0f;
//        CameraObserve::Rect2WH(body.roi, 1.0f, body_w, body_h);
//        LOGI("v3World width: %.2f, height: %.2f", body_w, body_h);

        LOGI("body (%d, %d, %d, %d), score: %.2f, orientation: %d", body.roi.x, body.roi.y, body.roi.width, body.roi.height, body.det_score, body.orientation);
        /* should do filter here */
        bodies.push_back(body);
    }
    return bodies;
}

float CvAlgoWrapper::CompareTwoPerson(const person_t& old_person, const person_t& new_person){
    const std::unordered_map<int, std::vector<float>>& new_feats = new_person.vTarget.vFeats;
    const std::unordered_map<int, std::vector<float>>& old_feats = old_person.vTarget.vFeats;
    float min_score = std::numeric_limits<float>::max();
    for(const auto& p : new_feats){
        int new_orient = p.first;
        const std::vector<float>& new_feat = p.second;
        if(new_feat.size() < 16){
            LOGE("WTF! new feature empty!");
            continue;
        }
        for(const auto& q : old_feats){
            int old_orient = q.first;
            const std::vector<float>& old_feat = q.second;
            if(old_feat.size() < 16){
                LOGE("WTF! old feature empty!");
                continue;
            }
            float score = pReid_ -> compute_feature_diff(old_feat, new_feat);
            LOGI("old_person[%d, %d][%d] compare with new_person[%d, %d][%d] score: %.2f", old_person.uid, old_person.rid, old_orient, new_person.uid, new_person.rid, new_orient, score);
            if(score < min_score){
                min_score = score;
            }
        }
    }
    LOGI("old_person[%d, %d] compare with new_person[%d, %d] min score: %.2f", old_person.uid, old_person.rid, new_person.uid, new_person.rid, min_score);
    return min_score;
}

bool CvAlgoWrapper::UpdateFeatures(const double now_t, const cv::Mat& raw, person_t& person){
    const double theta = person.states[0];
    const double dist = person.states[1];
    if(dist > GlobalConstants::VISUAL_FEATURE_UPDATE_M_ && fabs(theta) < GlobalConstants::VISUAL_FEATURE_UPDATE_DEG_ && IsPersonLonely(person)){
        VisualTarget_t& target = person.vTarget;
//        target.feats.clear();
        cv::Mat body(raw, target.roi);
        std::vector<float> probs;
        std::vector<float> feats;
        if(body.channels() == 1){
            cv::cvtColor(body, body, cv::COLOR_GRAY2BGR);
        }
        int orientation = pBodyPose_->classify(body, probs);
        pReid_ -> compute_feature(body, feats);
        target.vFeats[orientation] = feats;
        target.feats_tick_s = now_t;
        return true;
    }else{
        return false;
    }
}

void CvAlgoWrapper::AddNewFeature2Queue(const double now_t, person_t& new_person){
    if(new_person.rid < 0){
        ReidAssignment(new_person);
    }

    if( new_person.rid < 0 ){
        new_person.rid = personCounter_;
        personCounter_ += 1;
        localReidQueue_[new_person.uid] = new_person;
    }else{
        bool rid_found = false;
        for(auto& p : localReidQueue_){
            const uint32_t old_uid = p.first;
            person_t& old_person = p.second;
            if(old_person.rid == new_person.rid){
                localReidQueue_.erase(old_uid);
                localReidQueue_[new_person.uid] = new_person;
                rid_found = true;
                break;
            }
        }
        if(!rid_found){
            localReidQueue_[new_person.uid] = new_person;
        }
    }
    LOGI("new person[%d, %d] add to localReidQueue_", new_person.uid, new_person.rid);
    /* erase oldest from queue if needed */
    if(localReidQueue_.size() > GlobalConstants::LOCAL_REID_QUEUE_SIZE_){
        double oldest_t = now_t;
        int32_t oldest_uid = -1;
        int32_t oldest_rid = -1;
        for(const auto p : localReidQueue_){
            double last_s = p.second.vTarget.feats_tick_s;
            if(last_s < oldest_t){
                oldest_t = last_s;
                oldest_uid = p.first;
                oldest_rid = p.second.rid;
            }
        }
        if(oldest_uid > 0){
            localReidQueue_.erase(oldest_uid);
            LOGI("erase oldest person[%d, %d] from localReidQueue_, disappear time: %.2f sec", oldest_uid, oldest_rid, (now_t - oldest_t));
        }
    }
}

void CvAlgoWrapper::MapLidar2Mono(const std::vector<std::tuple<float, float>>& lidarRawPoints){
    vLidarPoints_.clear();
    for(int idx = 0; idx < lidarRawPoints.size(); ++idx){
        const std::tuple<float, float>& l = lidarRawPoints[idx];
        LidarPoint_t point;
        point.raw.deg = std::get<0>(l);
        point.raw.rad = std::get<0>(l) / 57.3f;
        point.raw.dist = std::get<1>(l);

        const float& sita = point.raw.rad;
        const float& r = point.raw.dist;
        Eigen::Vector3f lxyz(r*sin(sita), 0.0f, r*cos(sita));
        Eigen::Vector3f mxyz = CameraObserve::World2Pixel(lxyz);
        point.inMono.x = (int) mxyz[0];
        point.inMono.y = (int) mxyz[1];
        point.inMono.z = r;
        if(point.inMono.x < 0 || point.inMono.x > GlobalConstants::IMAGE_WIDTH_ || point.inMono.y < 0 || point.inMono.y > GlobalConstants::IMAGE_HEIGHT_){
            point.inMono.invisible = true;
        }
        vLidarPoints_.emplace_back(point);
    }
    /* arrange points in order */
    std::sort(vLidarPoints_.begin(), vLidarPoints_.end(), LidarPointComparison);
//    for(int i = 0; i < vLidarPoints_.size(); ++i){
//        LOGI("vLidarPoints [%d]: %.2f", i, vLidarPoints_[i].raw.deg);
//    }
//    LOGI("valid lidar points: %d", vLidarPoints_.size());
}

LidarCloud_t& CvAlgoWrapper::GetLidarCloud() {
    return vLidarPoints_;
}

bool CvAlgoWrapper::IsRoiValid(const cv::Rect& roi){
    if(roi.area() < 1000){
        LOGW("person too small, ignore");
        return false;
    }

    if( LidarObserve::RoiFloatingOnLaser(vLidarPoints_, roi) ){
        LOGW("floating person, ignore");
        return false;
    }
    return true;
}

bool CvAlgoWrapper::IsPersonLonely(const person_t& person){
    for(const auto& p : mHerds_){
        const uint32_t uid = p.first;
        const person_t& other = p.second;
        if(uid == person.uid){
            continue;
        }
        const cv::Rect& this_roi = person.vTarget.roi;
        const cv::Rect& other_roi = other.vTarget.roi;
        double iou = CameraObserve::IouOfRoi(this_roi, other_roi);
        if(iou > GlobalConstants::VISUAL_IOU_ISOLATE_){
            return false;
        }
    }
    return true;
}

bool CvAlgoWrapper::IsFulcrumInside(const person_t& person){
    bool rc = person.vTarget.roi.contains(person.fulcrum);
    if(!rc){
        LOGW("tracker[%d] lost fulcrum", person.uid);
    }
    return rc;
}

void CvAlgoWrapper::VisualAssignment(std::vector<VisualTarget_t>& newcomers){
    const int rows = newcomers.size();
    const int cols = mHerds_.size();
    cv::Mat cost(rows, cols, CV_32SC1);
    /* newcomers stand on rows, trackers stand on cols */
    for(int r = 0; r < rows; ++r){
        const std::vector<std::pair<uint32_t, double>>& ious = newcomers[r].iou_scores;
        for(int c = 0; c < cols; ++c){
            int score = (1 - ious[c].second) * 100;
            cost.at<int>(r, c) = score;
        }
    }
    std::vector<std::pair<int, int>> res = pKM_ -> compute(cost);
    for(int i = 0; i < res.size(); ++i){
        std::pair<int, int>& p = res[i];
        int r = p.first;
        int c = p.second;
        double iou = 1 - cost.at<int>(r, c) * 0.01f;
        uint32_t uid = newcomers[r].iou_scores[c].first;
        if(iou > GlobalConstants::VISUAL_IOU_MATCH_MIN_){
            person_t& person = mHerds_[uid];
            person.vTarget.update(newcomers[r]);
            person.visual_match = true;
            LOGI("[iou match] %d -> %d (uid = %d), iou: %.2f", r, c, uid, iou);
        }else{
            LOGI("[iou miss] %d -> %d (uid = %d), iou: %.2f", r, c, uid, iou);
        }
    }
}

void CvAlgoWrapper::LaserAssignment(const std::vector<LaserTarget_t>& newcomers){
    const int rows = newcomers.size();
    const int cols = mHerds_.size();
    cv::Mat cost(rows, cols * 2, CV_32SC1);
    /* newcomers stand on rows, trackers stand on cols */
    for(int r = 0; r < rows; ++r){
        const std::vector<std::pair<uint32_t, double>>& scores = newcomers[r].dist_scores;
        for(int c = 0; c < cols; ++c){
            /* same score for both left & right leg, unit: mm */
            int score = (int)(scores[c].second * 1000);
            cost.at<int>(r, 2*c) = score;
            cost.at<int>(r, 2*c+1) = score;
        }
    }
    std::vector<std::pair<int, int>> res = pKM_ -> compute(cost);
    for(int i = 0; i < res.size(); ++i){
        std::pair<int, int>& p = res[i];
        int r = p.first;
        int c = p.second;
        double diff = cost.at<int>(r, c) * 0.001f;
        uint32_t cc = (uint32_t)(c/2);
        uint32_t uid = newcomers[r].dist_scores[cc].first;
        if(mHerds_.find(uid) == mHerds_.end()){
            LOGE("WTF! tracker uid %d not found, r = %d, c = %d, cc = %d, rows = %d, cols = %d, scores = %d", uid, r, c, cc, rows, cols, newcomers[r].dist_scores.size());
            continue;
        }
        const LaserTarget_t& leg = newcomers[r];

        if(diff < GlobalConstants::LIDAR_TARGETS_NEAR_M_){
            person_t& person = mHerds_[uid];
            if(person.laser_match){
                person.lTarget.merge(leg);
                LOGI("[right leg match] %d (%.2f, %.2f) -> %d (uid = %d), diff: %.2f", r, leg.center_theta, leg.center_dist, c, uid, diff);
            }else{
                person.lTarget = leg;
                person.laser_match = true;
                LOGI("[left leg match] %d (%.2f, %.2f) -> %d (uid = %d), diff: %.2f", r, leg.center_theta, leg.center_dist, c, uid, diff);
            }
        }else{
            LOGI("[leg miss] %d (%.2f, %.2f) -> %d (uid = %d), diff: %.2f", r, leg.center_theta, leg.center_dist, c, uid, diff);
        }
    }
}

/* find rid for new person */
void CvAlgoWrapper::ReidAssignment(person_t& newp){
    const int cols = localReidQueue_.size();
    if(cols < 1){
        return;
    }
    const int rows = mHerds_.size() + 1;

    std::vector<uint32_t> new_people;
    for(const auto& p : mHerds_){
        new_people.emplace_back(p.first);
    }

    std::vector<uint32_t> old_people;
    for(const auto& q : localReidQueue_){
        old_people.emplace_back(q.first);
    }

    new_people.emplace_back(newp.uid);
    cv::Mat cost(rows, cols, CV_32SC1, std::numeric_limits<int16_t>::max());

    for(int r = 0; r < rows; ++r){
        const uint32_t new_uid = new_people[r];
        const person_t& new_person = (new_uid == newp.uid) ? newp : mHerds_[new_uid];
        const bool fresh = localReidQueue_.find(new_uid) != localReidQueue_.end();
        for(int c = 0; c < cols; ++c){
            const uint32_t old_uid = old_people[c];
            person_t& old_person = localReidQueue_[old_uid];
            /* for fresh targets, just set cross to zero */
            if(fresh){
                if(old_uid == new_uid){
                    cost.at<int>(r, c) = 0;
                    break;
                }
            }else{
                float score = CompareTwoPerson(old_person, new_person);
                cost.at<int>(r, c) = (int)(score * 100);
            }
        }
    }

    std::vector<std::pair<int, int>> res = pKM_ -> compute(cost);
    LOGI("[reid match] cost matrix %d * %d", rows, cols);
    for(int i = 0; i < res.size(); ++i){
        std::pair<int, int>& p = res[i];
        int r = p.first;
        int c = p.second;
        double reid_score = cost.at<int>(r, c) * 0.01f;
        uint32_t new_uid = new_people[r];
        uint32_t old_uid = old_people[c];
        person_t& new_person = (new_uid == newp.uid) ? newp : mHerds_[new_uid];
        const person_t& old_person = localReidQueue_[old_uid];
        LOGI("[reid match] %d (new_uid = %d) -> %d (old_uid = %d), reid_score: %.2f, rid = %d", r, new_uid, c, old_uid, reid_score, old_person.rid);
        if(reid_score < GlobalConstants::VISUAL_REID_MATCH_SCORE_){
            new_person.rid = old_person.rid;
        }
    }
}

void CvAlgoWrapper::WashHerds(){
    std::vector<uint32_t> zombies;
    for(auto& p : mHerds_){
        uint32_t uid = p.first;
        person_t& person = p.second;
        /* tracker quit mechanism */
        if(person.visual_disappear > GlobalConstants::VISUAL_DISAPPEAR_MIN_
        || person.laser_disappear > GlobalConstants::LASER_DISAPPEAR_MIN_
        || !IsFulcrumInside(person))
        {
            zombies.emplace_back(uid);
            LOGW("put tracker %d into zombie", uid);
        }
        person.visual_match = false;
        person.laser_match = false;
    }

    /* delete disappear trackers */
    for(auto idx : zombies){
        mHerds_.erase(idx);
        LOGI("erase tracker[%d]", idx);
    }

}

bool CvAlgoWrapper::AddNewTracker(const cv::Mat& raw, const VisualTarget_t& vBody){
    person_t person;

    Interval_t ivl;
    /* maybe a littel wider? */
    ivl.startPixel = vBody.roi.x;
    ivl.endPixel = vBody.roi.x + vBody.roi.width;
    ivl.domain = SenseDomain_t::VISUAL;
    LOGI("pick legs from interval %d -> %d", ivl.startPixel, ivl.endPixel);
    int rc = LidarObserve::PickLegsFromInterval(vLidarPoints_, ivl, person.lTarget);
    if(rc < 0){
        LOGE("failed to cluster legs, discard this roi");
        return false;
    }
    LOGI("legs center: %.2f, dist: %.2f", person.lTarget.center_theta, person.lTarget.center_dist);

    /* init ukf */
    std::shared_ptr<UKFBodyTracker> pkf = std::make_shared<UKFBodyTracker>();
    const double theta = person.lTarget.center_theta;
    const double dist = person.lTarget.center_dist;
    const double Z = dist * cos(theta / 57.3f);
    double width = CameraObserve::GetVisualWidth(vBody.roi, Z);
    /* ratio is pure visual element */
    double ratio = (double)vBody.roi.height / vBody.roi.width;
    double time_s = DepthUtils::CurrentMicros() * 1e-6;
    pkf -> CreateKF(UKFBodyTracker::KalmanFilterInit(theta, dist, width, ratio), time_s);

    person.pKf = pkf;
    person.uid = trackerCounter_;
    person.vTarget = vBody;
    UpdateStates(person);
    bool feature_ready = UpdateFeatures(time_s, raw, person);

    /* add to queue, only if features ready */
    if( feature_ready ){
        AddNewFeature2Queue(time_s, person);
    }
    mHerds_[trackerCounter_] = std::forward<person_t> (person);

    LOGI("add new tracker[%d], theta: %.2f, dist: %.2f", person.uid, theta, dist);
    ++ trackerCounter_;
    return true;
}

int CvAlgoWrapper::InitHerds(const cv::Mat& raw){
    mHerds_.clear();
    std::vector<BBox> bboxes = pDetector_ -> detect(raw);
    for(auto b : bboxes){
        VisualTarget_t vBody;
        vBody.roi = bbox2rect(b);
        vBody.det_score = b.score;
        if(IsRoiValid(vBody.roi)){
            AddNewTracker(raw, vBody);
        }
    }
    return mHerds_.size();
}

int CvAlgoWrapper::TrackHerds(const cv::Mat& raw){
    WashHerds();
    double time_s = DepthUtils::CurrentMicros() * 1e-6;
    std::vector<BBox> bboxes = pDetector_ -> detect(raw);
    /* ukf predict */
    for(auto& p : mHerds_){
        uint32_t uid = p.first;
        person_t& person = p.second;
        LOGI("predict tracker: %d", uid);

        UKFBodyTracker::State_t old_ss = person.pKf -> GetStates();
        const double old_theta = old_ss[UKFBodyTracker::kTheta];
        const double old_dist = old_ss[UKFBodyTracker::kDist];
        person.pKf -> Predict(time_s);
        UpdateStates(person);
        person.visual_disappear += 1;
        person.laser_disappear += 1;
        UKFBodyTracker::State_t new_ss = person.pKf -> GetStates();
        const double new_theta = new_ss[UKFBodyTracker::kTheta];
        const double new_dist = new_ss[UKFBodyTracker::kDist];
        const double new_width = new_ss[UKFBodyTracker::kWidth];
        const double new_ratio = new_ss[UKFBodyTracker::kRatio];

        cv::Rect new_roi;
        CameraObserve::Polar2Rect(old_theta, old_dist, person.vTarget.roi, new_theta, new_dist, new_width, new_ratio, new_roi);
        /* update visual roi after predict */
        person.vTarget.roi = new_roi;
    }

    /* laser assignment */
    std::vector<LaserTarget_t> laser_targets;
    for(auto& p : mHerds_){
        uint32_t uid = p.first;
        person_t& person = p.second;
        const double theta = person.states[0];
        const double dist = person.states[1];
        const double width = person.states[2];
        Interval_t ivl = LidarObserve::GetTrackerInterval(theta, dist, width);
        std::vector<LaserTarget_t> targets = LidarObserve::ClusterLidarInterval(vLidarPoints_, ivl);

        for(int i = 0; i < targets.size(); ++i){
            if( LidarObserve::TryFuseNewTarget(laser_targets, targets[i]) ){
                LOGI("LidarObserve tracker[%d] fused target[%d]", uid, i);
                continue;
            }

            const double theta1 = targets[i].center_theta;
            const double dist1 = targets[i].center_dist;
            std::vector<std::pair<uint32_t, double>>& scores = targets[i].dist_scores;
            int near_counter = 0;
            for(auto p : mHerds_){
                uint32_t uid = p.first;
                const double theta2 = p.second.states[0];
                const double dist2 = p.second.states[1];
                const double pdist_m = LidarObserve::PolarDistance(theta1, dist1, theta2, dist2);
                scores.emplace_back(uid, pdist_m);
                if(pdist_m < GlobalConstants::LIDAR_TARGETS_FAR_M_){
                    near_counter += 1;
                }
            }
            /* make sure it is not background */
            if( near_counter > 0 ){
                laser_targets.emplace_back(targets[i]);
            }
        }
    }
//    LOGI("valid laser targets size: %d", laser_targets.size());
    LidarObserve::WashLaserTargets(laser_targets);
    if(laser_targets.size() > 0){
        LaserAssignment(laser_targets);
        /* process observe for those assigned targets */
        for(auto& p : mHerds_){
            uint32_t uid = p.first;
            person_t& person = p.second;
            if(person.laser_match){
                const double theta = person.lTarget.center_theta;
                const double dist = person.lTarget.center_dist;
                person.pKf -> ObserveLidar(time_s, theta, dist);
                UpdateStates(p.second);
                person.laser_disappear = 0;
                LOGI("update laser observe, theta: %.2f, dist: %.2f", theta, dist);
            }
        }
    }

    /* visual assignment */
    std::vector<VisualTarget_t> visual_recall;
    std::vector<VisualTarget_t> visual_isolate;

    for(auto b : bboxes) {
        VisualTarget_t new_t;
        new_t.roi = bbox2rect(b);
        if(!IsRoiValid(new_t.roi)){
            continue;
        }

        /* create new tracker for isolate roi */
        int nice_count = 0;
        for(auto p : mHerds_){
            uint32_t uid = p.first;
            VisualTarget_t& old_t = p.second.vTarget;
            double iou = CameraObserve::IouOfRoi(new_t.roi, old_t.roi);
            new_t.iou_scores.emplace_back(uid, iou);
            if(iou > GlobalConstants::VISUAL_IOU_ISOLATE_){
                nice_count += 1;
            }
        }
        if(nice_count < 1){
            visual_isolate.emplace_back(new_t);
        }else{
            visual_recall.emplace_back(new_t);
        }
    }

    if(visual_recall.size() > 0){
        VisualAssignment(visual_recall);
        /* process observe for those assigned targets */
        for(auto& p : mHerds_){
            uint32_t uid = p.first;
            person_t& person = p.second;
            if(person.visual_match){
                const double theta = CameraObserve::GetVisualTheta(person.vTarget.roi);
                if(std::isnan(theta)){
                    LOGE("theta degenerate!");
                    continue;
                }
                person.visual_disappear = 0;
                UKFBodyTracker::State_t new_ss = person.pKf -> GetStates();
                const double dist = new_ss[UKFBodyTracker::kDist];
                const double Z = dist * cos(theta / 57.3f);
                const double width = CameraObserve::GetVisualWidth(person.vTarget.roi, Z);
                const double ratio = (double)person.vTarget.roi.height / (person.vTarget.roi.width + 1);
                person.pKf -> ObserveVisual(time_s, theta, width, ratio);
                UpdateStates(p.second);
                LOGI("update visual observe, theta: %.2f, width: %.2f, ratio: %.2f, dist: %.2f, Z: %.2f", theta, width, ratio, dist, Z);

                /* update feature */
                if(time_s - person.vTarget.feats_tick_s > GlobalConstants::VISUAL_FEATURE_UPDATE_S_){
                    LOGI("update feature for tracker[%d], time_s: %.2f, last_tick: %.2f", uid, time_s, person.vTarget.feats_tick_s);
                    if( UpdateFeatures(time_s, raw, person) ){
                        AddNewFeature2Queue(time_s, person);
                    }
                }
            }
        }
    }

    if(visual_isolate.size() > 0){
        for(const auto& t : visual_isolate){
            AddNewTracker(raw, t);
        }
    }

    return mHerds_.size();
}

Herds_t CvAlgoWrapper::GetHerds() const {
//    for(auto p : mHerds_){
//        uint32_t uid = p.first;
//        person_t& person = p.second;
//        std::vector<double>& ss = person.states;
//        LOGI("Herds[%d] ss: %d", uid, ss.size());
//    }
    return mHerds_;
}

void CvAlgoWrapper::UpdateStates(person_t& person){
    uint32_t uid = person.uid;
    if(person.pKf != nullptr){
        UKFBodyTracker::State_t ss = person.pKf -> GetStates();
        const double theta = ss[UKFBodyTracker::kTheta];
        const double dist = ss[UKFBodyTracker::kDist];
        const double width = ss[UKFBodyTracker::kWidth];
        const double ratio = ss[UKFBodyTracker::kRatio];
        const double omega = ss[UKFBodyTracker::kOmega];
        const double vel = ss[UKFBodyTracker::kVel];
        person.states.clear();
        person.states.emplace_back(theta);
        person.states.emplace_back(dist);
        person.states.emplace_back(width);
        person.states.emplace_back(ratio);
        person.states.emplace_back(omega);
        person.states.emplace_back(vel);
        person.fulcrum = CameraObserve::GetFulcrum(theta, dist);
        LOGI("Herds[%d] theta: %.2f, dist: %.2f, width: %.2f, ratio: %.2f, omega: %.2f, vel: %.2f", uid, theta, dist, width, ratio, omega, vel);
    }
}

}