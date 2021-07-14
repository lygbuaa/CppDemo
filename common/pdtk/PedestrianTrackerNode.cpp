//
// Created by hugoliu on 21-4-21.
//

#include "PedestrianTrackerNode.h"

namespace PedestrianTracker {
void PedestrianTrackerNode::Start() {
    pAlgo_ = std::unique_ptr<CvAlgoWrapper> (new CvAlgoWrapper());
    pAlgo_ -> LoadModels();

    alive_ = true;
    pMono_ = std::unique_ptr<unsigned char>(
            new unsigned char[GlobalConstants::IMAGE_HEIGHT_ * GlobalConstants::IMAGE_WIDTH_]);
    pWorkThread_ = std::unique_ptr<std::thread>(
            new std::thread(&PedestrianTrackerNode::WorkRoutine, this));
    pDataHelper_ = std::unique_ptr<DataHelper>(new DataHelper());
    OpenSensorHub();

//    AlgoUnitTest::LoadModels();
//    AlgoUnitTest::RunDetectAndReid();
//    AlgoUnitTest::InitKcf();
//    AlgoUnitTest::RunKcf();
}

void PedestrianTrackerNode::Stop() {
    alive_ = false;
    if (pWorkThread_) {
        pWorkThread_->join();
    }
    pSensorHub_ -> Stop();
}

bool PedestrianTrackerNode::OpenSensorHub() {
    pSensorHub_ = std::unique_ptr<UartSensorhubDriver> (new UartSensorhubDriver());
    if( pSensorHub_ -> Initialize() >= 0){
        pSensorHub_ -> RegisterLidarCallback(
            [&](std::vector<std::tuple<float, float>>&& lidarPoints) -> void {
                if(inPlayback_){
                    return;
                }
                std::unique_lock<std::mutex> lock(lidarMutex_);
                lidarRawPoints_ = lidarPoints;
                lidarCond_.notify_all();
            }
        );
        pSensorHub_ -> Start();
        return true;
    }
    return false;
}

int PedestrianTrackerNode::CaptureMono(const char *path) {
//    std::unique_lock<std::mutex> lock(monoMutex_);
    cv::Mat gray(GlobalConstants::IMAGE_HEIGHT_, GlobalConstants::IMAGE_WIDTH_, CV_8UC1,
                 reinterpret_cast<void *>(pMono_.get()));
    cv::imwrite(path, gray);
    return 0;
}

void PedestrianTrackerNode::SetTrack(bool enable){
    if(enable){
        data_.mode = WorkingMode_t::SINGLE_TRACKING;
        data_.try_to_focus = true;
        LOGI("[WorkingMode_t] toggle single tracking mode");
    }else{
        data_.mode = WorkingMode_t::MULTI_TRACKING;
        data_.try_to_focus = false;
        data_.focus_id = -1;
        LOGI("[WorkingMode_t] toggle multi tracking mode");
    }
}

void PedestrianTrackerNode::SetRecord(bool enable){
    if(enable){
        pDataHelper_ -> InitWriter();
    }else{
        pDataHelper_ -> FinishWriter();
    }
    inRecording_ = enable;
}

void PedestrianTrackerNode::SetPlayback(bool enable){
    if(enable){
        pDataHelper_ -> InitReader();
        data_.frame_id = 0;
    }else{
        pDataHelper_ -> FinishReader();
    }
    inPlayback_ = enable;
}

void PedestrianTrackerNode::Gray2Preview(const cv::Mat& gray) {
    cv::Mat rgba(gray.size(), CV_8UC4);
    cv::cvtColor(gray, rgba, cv::COLOR_GRAY2RGBA);
    char buffer[64] = {0};
    sprintf(buffer, "%ld  fps: %.1f", data_.frame_id, data_.fps);
    cv::Scalar text_color(240, 128, 0, 255);
    cv::putText(rgba, buffer, cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2);

    /* draw bodies */
    cv::Scalar body_color(128, 128, 128, 128);

    for(const auto p : data_.herds) {
        uint32_t uid = p.first;
        const person_t &person = p.second;
        int32_t rid = person.rid;
        const VisualTarget_t &body = person.vTarget;

        if(person.visual_match && person.laser_match){
            body_color = cv::Scalar (0, 0, 255, 255); //blue
        }else if(person.visual_match){
            body_color = cv::Scalar (255, 255, 0, 255); //yellow
        }else if(person.laser_match){
            body_color = cv::Scalar (255, 0, 0, 255); //red
        }

        int thickness = 1;
        if(data_.focus_id == p.first){
            thickness = 3;
//            body_color = cv::Scalar (0, 0, 255, 255);
        }

        cv::rectangle(rgba, body.roi, body_color, thickness);
        const double theta = person.states[0];
        const double dist = person.states[1];

        /* draw fulcrum */
        int green = (int)(dist/5.0f*255);
        cv::Scalar lidar_color(255-green, green, 0, 255);
        cv::circle(rgba, person.fulcrum, 6, body_color, 2);

        char buffer[64] = {0};
        sprintf(buffer, "id %d, %d", uid, rid);
        cv::putText(rgba, buffer, cv::Point(body.roi.x, body.roi.y+15), cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1);
        memset(buffer, 0, 64);
        sprintf(buffer, "%.0f, %.1f m", theta, dist);
        cv::putText(rgba, buffer, cv::Point(body.roi.x, body.roi.y+30), cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1);
    }

    /* draw lidar points */
    {
        std::unique_lock<std::mutex> lock(lidarMutex_);
        LidarCloud_t vLidarPoints = pAlgo_ -> GetLidarCloud();
        for(auto p : vLidarPoints){
            if(p.inMono.invisible){
                continue;
            }
            int green = (int)(p.inMono.z/5.0f*255);
            cv::Scalar lidar_color(255-green, green, 0, 255);
            int radius = 1;
//            if(p.raw.deg < 0.1f && p.raw.deg > -0.1f){
//                radius = 6;
//            }

//            if(p.raw.deg > 9.7f && p.raw.deg < 10.5f){
//                char buffer[16] = {0};
//                text_color = cv::Scalar(240, 40, 40, 255);
//                sprintf(buffer, "+");
//                cv::putText(rgba, buffer, cv::Point(p.inMono.x, p.inMono.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 1);
//            }else if(p.raw.deg > -10.5f && p.raw.deg < -9.7f){
//                char buffer[16] = {0};
//                text_color = cv::Scalar(240, 40, 40, 255);
//                sprintf(buffer, "-");
//                cv::putText(rgba, buffer, cv::Point(p.inMono.x, p.inMono.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 1);
//            }

            cv::circle(rgba, cv::Point2d(p.inMono.x, p.inMono.y), radius, lidar_color, 2);
        }
    }
    pCamera_->renderVideo(rgba.data);
    /* auto save playback results */
    if(inPlayback_){
        std::string output_path(GlobalConstants::CAPTURE_PATH_);
//        DepthUtils::CheckAndMkdir(output_path);
        char tmp_id[16] = {0};
        sprintf(tmp_id, "autosave.%06d.jpeg", data_.frame_id);
//        output_path += DepthUtils::GetTimestrMiliSec();
        output_path += tmp_id;
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        cv::imwrite(output_path, bgr);
    }
}

void PedestrianTrackerNode::WorkRoutine() {
    LOGI("work routine launch");
    int64_t last_frame_id = 0;
    long tick_time = DepthUtils::CurrentMicros();
    /* set raw image */
    cv::Mat mono_image(GlobalConstants::IMAGE_HEIGHT_, GlobalConstants::IMAGE_WIDTH_, CV_8UC1,
                reinterpret_cast<void *>(pMono_.get()));

    while (alive_) {
        int rc = -1;
        cv::Mat raw;
        if(inPlayback_){
            usleep(100*1000);
            /* read data from GlobalConstants::RECORD_PATH_ */
            std::unique_lock<std::mutex> lock(lidarMutex_);
            rc = pDataHelper_ -> Read(raw, lidarRawPoints_);
        }else{
            std::unique_lock<std::mutex> lock(lidarMutex_);
            lidarCond_.wait_for(lock, std::chrono::milliseconds(100));
            rc = pCamera_->getMonoData(pMono_.get());
            raw = mono_image;
        }
        if (rc < 0) {
            usleep(10 * 1000);
            continue;
        }

        {
            std::unique_lock<std::mutex> lock(lidarMutex_);
            pAlgo_ -> MapLidar2Mono(lidarRawPoints_);
        }

        /* track herds whatever happen */
        if(data_.init_done){
            pAlgo_ -> TrackHerds(raw);
            data_.herds = pAlgo_ -> GetHerds();
            if(data_.try_to_focus){
//                LOGI("try to focus: %d", data_.herds.size());
                double dist_min = 10.0f;
                int32_t follow_me = -1;
                for(auto& p : data_.herds){
                    uint32_t uid = p.first;
                    person_t& person = p.second;
                    double dist = person.states[1];
//                    LOGI("try to focus %d, %.2f", uid, dist);
                    if(dist < dist_min){
                        dist_min = dist;
                        follow_me = uid;
                    }
                }
                if(follow_me >= 0){
                    data_.focus_id = follow_me;
                    LOGI("start follow tracker %d, dist: %.2f", follow_me, dist_min);
                    data_.try_to_focus = false;
                }
            }

        }else{
            /* just init herds once */
            if(pAlgo_ -> InitHerds(raw) > 0){
                data_.init_done = true;
                data_.herds = pAlgo_ -> GetHerds();
            }
        }

        /* calc frame rate */
        ++ data_.frame_id;
        long now_time = DepthUtils::CurrentMicros();
        float interval = (now_time - tick_time) * 1e-6;
        if (interval > 1.0f) {
            data_.fps = (data_.frame_id - last_frame_id) / interval;
            last_frame_id = data_.frame_id;
            tick_time = now_time;
        }
        if(data_.frame_id % 30 == 0){
            LOGI("frame_id: %ld, fps: %.1f", data_.frame_id, data_.fps);
        }

        Gray2Preview(raw);

        if(inRecording_){
            std::unique_lock<std::mutex> lock(lidarMutex_);
            pDataHelper_ -> Write(raw, lidarRawPoints_);
        }
    }
    LOGI("work routine exit");
}
}
