#ifndef DEPTH_UTILS_H
#define DEPTH_UTILS_H

#include "glogging_adapter.h"
#include <sys/vfs.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <mutex>
#include <thread>
#include <memory>
#include <chrono>
#include <cstring>
#include <condition_variable>
#include <exception>
//#include <execinfo.h>
#include <cxxabi.h>
#include <dlfcn.h>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <memory>

namespace HugoLiuGithub
{
//using namespace std;
#define HANG_STOPWATCH() auto _DepthUtilsPtr_ = DepthUtils::HangStopWatch(__FUNCTION__);
#define MOVING_STOPWATCH(N) auto _DepthUtilsPtr_ = DepthUtils::MovingAverageStopWatch(__FUNCTION__, N);

class DepthUtils
{
private:

public:
    DepthUtils(){}
    ~DepthUtils(){}

    static inline __attribute__((always_inline)) uint64_t CurrentMicros() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::time_point_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now()).time_since_epoch()).count();
    }

    static std::string ShellCall(std::string cmd, const char* type = "r");
    static bool CheckFileExist(const char* path);
    static int32_t CheckAndMkdir(std::string dir);
    static void MakeDir(std::string path);
    static std::string Micro2Date(int64_t micros_sec);
    static std::string GetTimestrMiliSec();
    static std::shared_ptr<uint64_t> HangStopWatch(const char* func_name);
    static std::shared_ptr<uint64_t> MovingAverageStopWatch(const char* func_name, const uint16_t moving_average_n = 100);
//    static std::string BackTrace(const int nMaxFrames = 32, const int skip = 1);
    static int GetBusIdFromUri(const std::string uri, int index, const char* delimiters = "@/");

};

}

#endif //DEPTH_UTILS_H