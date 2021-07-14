#include "DepthUtils.h"

namespace PedestrianTracker
{
//using namespace std;

    /*run shell cmd, and get the return value*/
    std::string DepthUtils::ShellCall(std::string cmd, const char* type){
        std::string result;
        FILE* fp = popen(cmd.c_str(), type);
        char buf[64] = {0};
        int32_t bytes_total = 0;
        int32_t bytes_read = 0;
        while( (bytes_read = fread(buf, 1, sizeof(buf), fp)) > 0 ){
            bytes_total += bytes_read;
            result.append(buf, bytes_read);
            memset(buf, 0, sizeof(buf));
        }

        pclose(fp);
        if(bytes_total <= 0){
            LOGW("WTF! ShellCall (%s) result is empty!", cmd.c_str());
        } else {
            LOGD("ShellCall (%s) result: %s", cmd.c_str(), result.c_str());
        }
        return result;
    }

    bool DepthUtils::CheckFileExist(const char* path){
        return ( access( path, F_OK ) == 0 );
    }

    int32_t DepthUtils::CheckAndMkdir(std::string dir){
        if (access(dir.c_str(), 0) == -1)
        {
            int flag=mkdir(dir.c_str(), 0777);
            if (flag == 0)
            {
                return 0;
            } else {
                return -1;
            }
        }
        return 0;
    }

    //create new dir
    void DepthUtils::MakeDir(std::string path){
        std::string cmd = "mkdir -p " + path;
        int32_t exit_code = system(cmd.c_str());
        LOGW("ShellCall (%s) exit code: %d", cmd.c_str(), exit_code);
    }

    /*convert micro seconds to date*/
    std::string DepthUtils::Micro2Date(int64_t micros_sec){
        char tmbuf[64] = {0};
        char mbuf[10] = {0};
        const uint32_t denom = 1e6;
        int64_t sec = micros_sec / denom;
        int32_t msec = micros_sec % denom;
        struct tm *tm = localtime(&sec);
        strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d-%H-%M-%S", tm);
        sprintf(mbuf, "-%03d", msec/1000);
        std::string date(tmbuf);
        date += mbuf;
        return date;
    }

    std::string DepthUtils::GetTimestrMiliSec() {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        struct tm * tm = localtime(&tv.tv_sec);
        char tmbuf[64] = {0};
        char mbuf[10] = {0};
        strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d-%H-%M-%S", tm);
        sprintf(mbuf, "-%03d", tv.tv_usec/1000);
        std::string date(tmbuf);
        date += mbuf;
        return date;
    }

    /*print function enter/exit/time_interval*/
    std::shared_ptr<uint64_t> DepthUtils::HangStopWatch(const char* func_name){
        uint64_t* pts_us = new uint64_t;
        *pts_us = CurrentMicros();
        LOGI("stop watch trigger by %s (epoch %ld us)", func_name, *pts_us);
        return std::shared_ptr<uint64_t>(pts_us, [func_name](uint64_t* ptr){
            uint64_t ts_us = CurrentMicros();
            // LOG(INFO) << "stop watch end: " << ts_us << " us.";
            LOGI("stop watch end by %s (elapse = %ld us)", func_name, (ts_us - *ptr));
            delete ptr;
        });
    }

    /*print function enter/exit/time_interval*/
    std::shared_ptr<uint64_t> DepthUtils::MovingAverageStopWatch(const char* func_name, const uint16_t moving_average_n){
        uint64_t* pts_us = new uint64_t;
        *pts_us = CurrentMicros();
//        LOGI("stop watch trigger by %s (epoch %ld us)", func_name, *pts_us);
        return std::shared_ptr<uint64_t>(pts_us, [=](uint64_t* ptr){
            static float moving_average_elapse = 0.0f;
            const float forget_factor = 1.0f/moving_average_n;
            uint64_t ts_us = CurrentMicros();
            // LOG(INFO) << "stop watch end: " << ts_us << " us.";
            moving_average_elapse = moving_average_elapse*(1-forget_factor) + (ts_us - *ptr)*forget_factor;
            LOGI("%s moving_average_elapse = %.1f us", func_name, moving_average_elapse);
            delete ptr;
        });
    }

    int DepthUtils::GetBusIdFromUri(const std::string uri, const int index, const char* delimiters){
        char str[64] = {0};
        memcpy(str, uri.c_str(), uri.size());
        char* pch;
        int idx = 0;
        int busId = -1;
        LOGI("extract %d-th part from: %s", index, str);
        pch = strtok (str, delimiters);
        while (pch != nullptr)
        {
            ++idx;
            if(idx == index){
                busId = atoi(pch);
                LOGI("found busid: %d", busId);
                break;
            }
            pch = strtok (nullptr, "@/");
        }
        return busId;
    }

#if 0
    /*dump the call-stack after exception*/
    std::string DepthUtils::BackTrace(const int nMaxFrames, const int skip){
        void *callstack[nMaxFrames];
        char buf[1024];
        int nFrames = backtrace(callstack, nMaxFrames);
        char **symbols = backtrace_symbols(callstack, nFrames);

        std::ostringstream trace_buf;
        for (int i = skip; i < nFrames; i++) {
            Dl_info info;
            if (dladdr(callstack[i], &info)) {
                char *demangled = NULL;
                int status;
                demangled = abi::__cxa_demangle(info.dli_sname, NULL, 0, &status);
                // LOG(INFO) << "symbols[" << i << "]: " << symbols[i];
                // LOG(INFO) << "info.dli_sname: " << info.dli_sname;
                // LOG(INFO) << "demangled: " << demangled << "stauts: " << status;
                snprintf(buf, sizeof(buf), "%-2d: %p\t%s\n",
                        i, callstack[i], status == 0 ? demangled : info.dli_sname);
                free(demangled);
            } else {
                snprintf(buf, sizeof(buf), "%-2d: %p\t%s\n", i, callstack[i], symbols[i]);
            }
            trace_buf << buf;
        }
        free(symbols);
        if (nFrames >= nMaxFrames)
            trace_buf << "[truncated]\n";
        return trace_buf.str();
}
#endif

}