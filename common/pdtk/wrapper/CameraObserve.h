//
// Created by hugoliu on 21-5-17.
//

#ifndef PEDESTRIANTRACKER_CAMERAOBSERVE_H
#define PEDESTRIANTRACKER_CAMERAOBSERVE_H

#include "GlobalConstants.h"
#include "GlobalTypes.h"
#include <Eigen/Eigen>
#include "utils/DepthUtils.h"
#include "Munkres.h"
#include <opencv2/opencv.hpp>

namespace PedestrianTracker {
class CameraObserve {
private:
    static Eigen::Matrix<float, 3, 3> r33L2C_;
    static Eigen::Matrix<float, 3, 3> r33L2CInv_;
    static Eigen::Vector3f t3L2C_;
    static Eigen::Matrix<float, 3, 3> m33Intrisics_;
    static Eigen::Matrix<float, 3, 3> m33IntrisicsInv_;

public:
    explicit CameraObserve(){
        InitParams();
    }
    CameraObserve(const CameraObserve&) = delete;
    ~CameraObserve(){}

    static void InitParams();
    static void FovDistortPoint(float& xc, float& yc);
    static void FovUndistortPoint(float& xc, float& yc);
    static Eigen::Vector3f World2Pixel(const Eigen::Vector3f& v3World);
    static Eigen::Vector3f Pixel2World(const Eigen::Vector3f& v3Pixel, const double Z);
    /* calc width & height in meter from cv::Rect & distance in meter */
    static void Polar2Rect(const double old_theta, const double old_dist, const cv::Rect& old_roi, const double new_theta, const double new_dist, const double new_width, const double new_ratio, cv::Rect& new_roi);
    static double IouOfRoi(const cv::Rect& roi1, const cv::Rect& roi2);
    static double GetVisualTheta(const cv::Rect& roi);
    static double GetVisualWidth(const cv::Rect& roi, const double Z);
    static double GetVisualHeight(const cv::Rect& roi, const double Z);
    static cv::Point2d GetFulcrum(const double theta, const double dist);
};
}

#endif //PEDESTRIANTRACKER_CAMERAOBSERVE_H
