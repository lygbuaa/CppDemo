//
// Created by hugoliu on 21-4-21.
//

#ifndef PEDESTRIANTRACKER_GLOBALCONSTANTS_H
#define PEDESTRIANTRACKER_GLOBALCONSTANTS_H

namespace PedestrianTracker {
namespace GlobalConstants {
    #define PIX_FORMATE_     V4L2_PIX_FMT_YUYV
    constexpr static int IMAGE_WIDTH_ = 640;
    constexpr static int IMAGE_HEIGHT_ = 480;
    constexpr static char *VIDEO_FILE_ = "/dev/video3";
    constexpr static char *TTY_FILE_ = "/dev/ttyACM0";
    constexpr static bool MINI_LIDAR_TRANSPOSE_ = true;
    constexpr static float LIDAR_CLUSTER_MIN_DIFF_M_ = 0.25f;
    constexpr static float LIDAR_CLUSTER_MIN_COUNT_ = 2;
    constexpr static float LIDAR_TARGETS_NEAR_M_ = 0.75f; //within this distance count for the same person
    constexpr static float LIDAR_TARGETS_FAR_M_ = 1.75f; //more than 1.8f count for background
    constexpr static float LIDAR_INTERVAL_INFLATE_ = 1.0f; //find laser target in larger interval
    constexpr static float LIDAR_TARGET_MIN_WIDTH_ = 0.05f; //very thin ankle
    constexpr static float LIDAR_TARGET_MAX_WIDTH_ = 0.45f; //maybe two legs abreast, or long-skirts-girl

//    constexpr static float VISUAL_REID_SIMILAR_SCORE_ = 1.2f;

    constexpr static float VISUAL_IOU_MATCH_MIN_ = 0.3f;
    constexpr static float VISUAL_IOU_ISOLATE_ = 0.05f;
    constexpr static float VISUAL_REID_MATCH_SCORE_ = 1.0f;
    constexpr static int LOCAL_REID_QUEUE_SIZE_ = 10;
    constexpr static float VISUAL_FEATURE_UPDATE_S_ = 1.0f; //update every 1.0 sec
    constexpr static float VISUAL_FEATURE_UPDATE_M_ = 1.0f; //update faraway than 1.0m
    constexpr static float VISUAL_FEATURE_UPDATE_DEG_ = 45.0f; //update between -45 ~ +45
    constexpr static int VISUAL_DISAPPEAR_MIN_ = 6;
    constexpr static int LASER_DISAPPEAR_MIN_ = 3;

    constexpr static char *ASSETS_PATH_ = "/storage/emulated/0/pdtk/assets/";
    constexpr static char *RECORD_PATH_ = "/storage/emulated/0/pdtk/record/";
    constexpr static char *CAPTURE_PATH_ = "/storage/emulated/0/pdtk/capture/";
    constexpr static char *CV_DETECTION_MODEL_ = "models/snpe_1v36/bp_retinanet_fbnet_7.2_0156000_quantized.snpe.json";
    constexpr static char *CV_BODYPOSE_MODEL_ = "models/snpe_1v25/people_pose_3type_quantized.json";
    constexpr static char *CV_BODYREID_MODEL_ = "models/snpe_1v36/pelee3486_lessiter_triplet_softmax_center_quantized.snpe.json";

/*
* X: along CMOS, point to right;
* Y: along CMOS, point to down;
* Z: normal to CMOS, point to front;
* pitch: around X, left hand;
* yaw: aound Y, left hand;
* roll: around Z, left hand;
* */
    constexpr static float L2M_PITCH_RAD_ = 18.0f / 57.3f;
    constexpr static float L2M_YAW_RAD_ = 0.0f;
    constexpr static float L2M_ROLL_RAD_ = 0.0f;
    constexpr static float L2M_X_M_ = 0.0f;
    constexpr static float L2M_Y_M_ = 0.15f;
    constexpr static float L2M_Z_M_ = 0.0f;
    constexpr static float M2G_Y_M_ = 0.3f; //lidar to ground

    constexpr static float MONO_FX_ = 271.729f;
    constexpr static float MONO_FY_ = 271.954f;
    constexpr static float MONO_CX_ = 340.0;
    constexpr static float MONO_CY_ = 240.0;
    constexpr static float MONO_OMEGA_ = 0.877f;
};
}

#endif //PEDESTRIANTRACKER_GLOBALCONSTANTS_H
