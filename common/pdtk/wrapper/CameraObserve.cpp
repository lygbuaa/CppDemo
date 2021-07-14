//
// Created by hugoliu on 21-5-17.
//
#include "CameraObserve.h"

namespace PedestrianTracker {
    Eigen::Matrix<float, 3, 3> CameraObserve::r33L2C_;
    Eigen::Matrix<float, 3, 3> CameraObserve::r33L2CInv_;
    Eigen::Vector3f CameraObserve::t3L2C_;
    Eigen::Matrix<float, 3, 3> CameraObserve::m33Intrisics_;
    Eigen::Matrix<float, 3, 3> CameraObserve::m33IntrisicsInv_;

    void CameraObserve::InitParams() {
        using namespace GlobalConstants;
        m33Intrisics_ << MONO_FX_, 0.0f, MONO_CX_,
                0.0f, MONO_FY_, MONO_CY_,
                0.0f, 0.0f, 1.0f;

        m33IntrisicsInv_ = m33Intrisics_.inverse();

        t3L2C_ << L2M_X_M_, L2M_Y_M_, L2M_Z_M_;

        Eigen::Matrix<float, 3, 3> dcm_pitch;
        dcm_pitch <<  1.0f,	0.0f, 0.0f,
                0.0f, cos(L2M_PITCH_RAD_), sin(L2M_PITCH_RAD_),
                0.0f, -1*sin(L2M_PITCH_RAD_), cos(L2M_PITCH_RAD_);

        Eigen::Matrix<float, 3, 3> dcm_yaw;
        dcm_yaw << cos(L2M_YAW_RAD_), 0.0f, -1*sin(L2M_YAW_RAD_),
                0.0f, 1.0f, 0.0f,
                sin(L2M_YAW_RAD_), 0.0f, cos(L2M_YAW_RAD_);

        Eigen::Matrix<float, 3, 3> dcm_roll;
        dcm_roll << cos(L2M_ROLL_RAD_), -1*sin(L2M_ROLL_RAD_), 0.0f,
                sin(L2M_ROLL_RAD_), cos(L2M_ROLL_RAD_), 0.0f,
                0.0f, 0.0f, 1.0f;

        r33L2C_ = dcm_pitch * dcm_yaw * dcm_roll;
        r33L2CInv_ = r33L2C_.inverse();

        std::stringstream ss;
        ss << "\nm33Intrisics_:\n" << m33Intrisics_;
        ss << "\nt3L2C_:\n" << t3L2C_;
        ss << "\nr33L2C_:\n" << r33L2C_;
        LOGI("InitParams: %s", ss.str().c_str());
    }

    /* using fov model, impose distortion,
     * xc & yc: point in camera orientation
     * ru: without distortion, rd: with distortion */
    void CameraObserve::FovDistortPoint(float& xc, float& yc){
        using namespace GlobalConstants;
        const float ru = sqrt(xc*xc + yc*yc);
        const float rd = atan(2*ru*tan(MONO_OMEGA_/2)) / MONO_OMEGA_;
        xc = rd * xc / ru;
        yc = rd * yc / ru;
    }

    /* using fov model, strip distortion
     * xc & yc: point in camera orientation
     * ru: without distortion, rd: with distortion */
    void CameraObserve::FovUndistortPoint(float& xc, float& yc){
        using namespace GlobalConstants;
        const float rd = sqrt(xc*xc + yc*yc);
        const float ru = tan(rd * MONO_OMEGA_) / (2 * tan(MONO_OMEGA_/2));
        xc = xc * ru / rd;
        yc = yc * ru / rd;
    }

    Eigen::Vector3f CameraObserve::World2Pixel(const Eigen::Vector3f& v3World){
        Eigen::Vector3f cxyz = r33L2C_ * (v3World + t3L2C_);
        cxyz /= cxyz[2];
        FovDistortPoint(cxyz[0], cxyz[1]);
        Eigen::Vector3f v3Pixel = m33Intrisics_ * cxyz;
        return v3Pixel;
    }

    Eigen::Vector3f CameraObserve::Pixel2World(const Eigen::Vector3f& v3Pixel, const double Z){
        Eigen::Vector3f cxyz = m33IntrisicsInv_ * (v3Pixel);
        FovUndistortPoint(cxyz[0], cxyz[1]);
        Eigen::Vector3f v3Camera(cxyz[0] * Z, cxyz[1] * Z, Z);
        Eigen::Vector3f v3Lidar = r33L2CInv_ * v3Camera - t3L2C_;
        return v3Lidar;
    }

    void CameraObserve::Polar2Rect(const double old_theta, const double old_dist, const cv::Rect& old_roi, const double new_theta, const double new_dist, const double new_width, const double new_ratio, cv::Rect& new_roi){
        /* calc center diff */
        using namespace GlobalConstants;
        const double old_rad = old_theta / 57.3f;
        const double new_rad = new_theta / 57.3f;
        Eigen::Vector3f old_center(old_dist*sin(old_rad), 0.0f, old_dist*cos(old_rad));
        Eigen::Vector3f new_center(new_dist*sin(new_rad), 0.0f, new_dist*cos(new_rad));
        Eigen::Vector3f poc = World2Pixel(old_center);
        Eigen::Vector3f pnc = World2Pixel(new_center);
        Eigen::Vector3f pdiff = pnc - poc;
        int x_diff = (int)pdiff[0];
        int y_diff = (int)pdiff[1];
        new_roi.x = old_roi.x + x_diff;
        new_roi.y = old_roi.y + y_diff;
        new_roi.width = old_roi.width;
        new_roi.height = old_roi.height;
//        Eigen::Vector3f new_right(new_center[0] + new_width/2*cos(new_rad), 0.0f, new_center[2] - new_width/2*sin(new_rad));
//        Eigen::Vector3f new_left(new_center[0] - new_width/2*cos(new_rad), 0.0f, new_center[2] + new_width/2*sin(new_rad));
//        Eigen::Vector3f pnr = World2Pixel(old_center);
//        Eigen::Vector3f pnl = World2Pixel(new_center);
//        new_roi.width = (int)fabs(pnr[0] - pnl[0]);
//        new_roi.height = (int)(new_roi.width * new_ratio);
//        LOGI("TrackHerds center diff  %.2f, %.2f", x_diff, y_diff);
    }

    double CameraObserve::IouOfRoi(const cv::Rect& roi1, const cv::Rect& roi2){
        double inter_area = (roi1&roi2).area();
        double union_area = (roi1|roi2).area();
        double iou = inter_area / union_area;
        iou = (iou > 1.0f) ? 1.0f : iou;
        return iou;
    }

    double CameraObserve::GetVisualTheta(const cv::Rect& roi){
        Eigen::Vector3f v3Pixel(roi.x + roi.width/2, roi.y + roi.height, 1.0f);
        Eigen::Vector3f cxyz = m33IntrisicsInv_ * v3Pixel;
        cxyz /= cxyz[2];
        FovUndistortPoint(cxyz[0], cxyz[1]);
        double theta = asin(cxyz[0]) * 57.3;
        return theta;
    }

    double CameraObserve::GetVisualWidth(const cv::Rect& roi, const double Z){
        Eigen::Vector3f pLD(roi.x, roi.y + roi.height, 1.0f);
        Eigen::Vector3f pRD(roi.x + roi.width, roi.y + roi.height, 1.0f);
        Eigen::Vector3f wLD = CameraObserve::Pixel2World(pLD, Z);
        Eigen::Vector3f wRD = CameraObserve::Pixel2World(pRD, Z);
        double W = wRD[0] - wLD[0];
        return W;
    }

    double CameraObserve::GetVisualHeight(const cv::Rect& roi, const double Z){
        Eigen::Vector3f pMD(roi.x + roi.width/2, roi.y + roi.height, 1.0f);
        Eigen::Vector3f pMU(roi.x + roi.width/2, roi.y, 1.0f);
        Eigen::Vector3f wMD = CameraObserve::Pixel2World(pMD, Z);
        Eigen::Vector3f wMU = CameraObserve::Pixel2World(pMU, Z);
        double H = wMD[0] - wMU[0];
        return H;
    }

    cv::Point2d CameraObserve::GetFulcrum(const double theta, const double dist){
        const double rad = theta / 57.3f;
        Eigen::Vector3f center(dist*sin(rad), 0.0f, dist*cos(rad));
        Eigen::Vector3f p = World2Pixel(center);
        return cv::Point2d((int)p[0], (int)p[1]);
    }


}
