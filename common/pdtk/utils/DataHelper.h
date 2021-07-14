//
// Created by hugoliu on 21-5-10.
//

#ifndef PEDESTRIANTRACKER_DATAHELPER_H
#define PEDESTRIANTRACKER_DATAHELPER_H

#include <iostream>
#include <fstream>
#include "utils/DepthUtils.h"
#include "utils/tinyjson.hpp"

namespace PedestrianTracker {

class DataHelper{
private:
    static constexpr char* JSON_FILENAME_ = "record_list.json";
    std::string lidarPath_{GlobalConstants::RECORD_PATH_};
    std::string monoPath_{GlobalConstants::RECORD_PATH_};
    std::string jsonPath_{GlobalConstants::RECORD_PATH_};
    std::ofstream fJsonWrite_;
    std::ifstream fJsonRead_;
    uint64_t wCnt_{0};
    uint64_t rCnt_{0};

public:
    DataHelper(){
        /* get dirs ready */
        lidarPath_ += "lidar/";
        DepthUtils::MakeDir(lidarPath_);
        monoPath_ += "mono/";
        DepthUtils::MakeDir(monoPath_);
        jsonPath_ += JSON_FILENAME_;
    }

    ~DataHelper(){
        FinishWriter();
        FinishReader();
    }

    void InitWriter(){
        /* get json file ready */
        fJsonWrite_.open(jsonPath_.c_str(), std::ofstream::out);
        wCnt_ = 0;
    }

    void FinishWriter(){
        if(fJsonWrite_.is_open()){
            fJsonWrite_.close();
        }
    }

    void Write(const cv::Mat& image, const std::vector<std::tuple<float, float>>& lidar){
        std::string time_str = DepthUtils::GetTimestrMiliSec();
        std::string mono_path = monoPath_ + time_str + ".jpg";
        cv::imwrite(mono_path, image);
        std::string lidar_path = lidarPath_ + time_str + ".lidar";
        std::ofstream ofs(lidar_path, std::ofstream::out);
        for(auto point:lidar){
            ofs << std::get<0>(point) << ", " << std::get<1>(point) << std::endl;
        }
        ofs.close();
        tiny::TinyJson obj;
        obj["time"].Set(time_str);
        obj["mono"].Set(mono_path);
        obj["lidar"].Set(lidar_path);
        std::string record = obj.WriteJson(false);
        fJsonWrite_ << record << std::endl;
        fJsonWrite_.flush();
        ++ wCnt_;

        LOGI("saved data [%ld]: %s", wCnt_, record.c_str());
    }

    void InitReader(){
        /* get json file ready */
        fJsonRead_.open(jsonPath_.c_str(), std::ifstream::in);
        rCnt_ = 0;
    }

    void FinishReader(){
        if(fJsonRead_.is_open()){
            fJsonRead_.close();
        }
    }

    int Read(cv::Mat& image, std::vector<std::tuple<float, float>>& lidar){
        if(fJsonRead_.is_open()){
            std::string line;
            std::getline(fJsonRead_, line);
            if(line.size() < 4){
                return -1;
            }
            ++ rCnt_;

            tiny::TinyJson obj;
            obj.ReadJson(line);
            std::string mono_path = obj.Get<std::string>("mono");
            std::string lidar_path = obj.Get<std::string>("lidar");
            LOGI("read data [%d] mono: %s, lidar: %s", rCnt_, mono_path.c_str(), lidar_path.c_str());
            image = cv::imread(mono_path, cv::IMREAD_GRAYSCALE);
            if(image.empty()){
                return -2;
            }

            std::ifstream ifs(lidar_path, std::ifstream::in);
            std::string ray;
            int32_t ray_cnt = 0;
            lidar.clear();

            if(ifs.is_open()) {
                while (std::getline(ifs, ray)) {
                    float angle = 0.0f;
                    float dist = 0.0f;
                    if (ray.size() < 2 || ray_cnt > 360) {
                        break;
                    }
                    if (sscanf(ray.c_str(), "%f, %f", &angle, &dist) < 0) {
                        break;
                    }
                    lidar.emplace_back(std::make_tuple(angle, dist));
                    ++ray_cnt;
                }
            }
            return ray_cnt;
        }else{
            return -3;
        }
    }

};

}
#endif //PEDESTRIANTRACKER_DATAHELPER_H
