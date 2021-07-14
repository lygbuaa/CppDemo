//
// Created by hugoliu on 19-9-24.
//

#ifndef VISIONSDK_GLOGGING_H
#define VISIONSDK_GLOGGING_H
#pragma once

#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <android/log.h>
#include <ctime>
#include <string>
#include <libgen.h>

#define __FILENAME__ (basename(__FILE__))
#define TAG  "PedestrianTracker"

#define CHECK(x) \
    if(!(x)) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_GT(x, y) \
    if( x <= y ) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_GE(x, y) \
    if( x < y ) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_LE(x, y) \
    if( x > y ) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_LT(x, y) \
    if( x >= y ) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_EQ(x, y) \
    if( x != y ) \
        LogMessageFatal(__FILENAME__, __LINE__).stream() << TAG << " check failed: " #x << ' '

#define CHECK_NEAR(val1, val2, margin)           \
  do {                                           \
    CHECK_LE((val1), (val2)+(margin));           \
    CHECK_GE((val1), (val2)-(margin));           \
  } while (0)


#define LOGD(format, ...) __android_log_print(ANDROID_LOG_DEBUG, TAG,\
        "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define LOGI(format, ...) __android_log_print(ANDROID_LOG_INFO, TAG,\
        "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define LOGW(format, ...) __android_log_print(ANDROID_LOG_WARN, TAG,\
        "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define LOGE(format, ...) __android_log_print(ANDROID_LOG_ERROR, TAG,\
        "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define DISABLE_OPTIONAL_LOG
#ifdef DISABLE_OPTIONAL_LOG
#define LOGIO(...)
#define LOGEO(...)
#define LOGDO(...)
#define LOGWO(...)
#else
#define LOGIO(...) LOGI(__VA_ARGS__)
#define LOGEO(...) LOGE(__VA_ARGS__)
#define LOGDO(...) LOGD(__VA_ARGS__)
#define LOGWO(...) LOGW(__VA_ARGS__)
#endif

class Error : public std::runtime_error {
public:
    explicit Error(const std::string& s) : std::runtime_error(s){}
};

class DateLogger{
public:
    DateLogger(){}
    const char* HumanDate(){
        time_t tv = time(NULL);
        struct tm now;
        localtime_r(&tv, &now);
        snprintf(_buffer, sizeof(_buffer), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);
        return _buffer;
    }
private:
    char _buffer[9];
};

class LogMessageFatal{
public:
    LogMessageFatal(const char *pcFile, int iLine){
        _log_stream << "[" << _pretty_date.HumanDate() << "]" << pcFile << ": " << iLine << ": ";
    }

    std::ostringstream &stream(){
        return _log_stream;
    }

    ~LogMessageFatal() noexcept(false){
        throw Error(_log_stream.str());
    }

private:
    std::ostringstream _log_stream;
    DateLogger _pretty_date;
};



#endif //VISIONSDK_GLOGGING_H
