//
// Created by hugoliu on 21-4-21.
//

#ifndef PEDESTRIANTRACKER_PEDESTRIANTRACKERNODE_H
#define PEDESTRIANTRACKER_PEDESTRIANTRACKERNODE_H

#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "V4L2/V4L2Camera.h"
#include "utils/glogging.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "GlobalConstants.h"
#include "utils/DepthUtils.h"
#include "utils/DataHelper.h"
#include "SensorHub/uart_sensorhub_driver.h"
#include "algo/test/AlgoUnitTest.h"
#include "algo/wrapper/CvAlgoWrapper.h"

namespace PedestrianTracker {

struct DisplayInfo_t{
    WorkingMode_t mode {WorkingMode_t::MULTI_TRACKING};
    volatile bool init_done {false};
    volatile bool try_to_focus {false};
    int64_t frame_id {0};
    float fps {0.0f};
    int32_t focus_id {-1};
    Herds_t herds;
};

class PedestrianTrackerNode {
private:
    std::unique_ptr<V4L2Camera> pCamera_ = nullptr;
    std::unique_ptr<std::thread> pWorkThread_ = nullptr;
    std::mutex monoMutex_;
    std::mutex lidarMutex_;
    std::condition_variable lidarCond_;
    std::unique_ptr<unsigned char> pMono_;
    volatile bool alive_ = false;
    DisplayInfo_t data_;
    std::unique_ptr<SensorhubDriver> pSensorHub_ = nullptr;
    std::unique_ptr<CvAlgoWrapper> pAlgo_ = nullptr;
    std::unique_ptr<DataHelper> pDataHelper_ = nullptr;
    std::vector<std::tuple<float, float>> lidarRawPoints_;

    volatile bool inRecording_ {false};
    volatile bool inPlayback_ {false};

public:
    PedestrianTrackerNode() {}

    ~PedestrianTrackerNode() {}

    void SetCameraPtr(V4L2Camera *ptr) {
        pCamera_.reset(ptr);
    }

    void Start();
    void Stop();
    void WorkRoutine();
    void Gray2Preview(const cv::Mat& gray);
    int CaptureMono(const char* path);

    bool OpenSensorHub();

    void SetTrack(bool enable);
    void SetRecord(bool enable);
    void SetPlayback(bool enable);
};
}
#endif //PEDESTRIANTRACKER_PEDESTRIANTRACKERNODE_H
