#ifndef GLOGGING_ADAPTER_H
#define GLOGGING_ADAPTER_H

/*
* set 0 for glog/logging.h, set 1 for android/log.h
*/
#define GLOG_1_ALOG_1 0

/*
* only print base file name
*/
#define __FILENAME__ (basename(__FILE__))

#if GLOG_0_ALOG_1 == 0
    /* set release mode */
    #ifndef NDEBUG
        #define NDEBUG 
    #endif

    #ifdef __cplusplus
        #include <memory>
        #include <glog/logging.h>
        /*
        * delete logs lower than this level, delete all logs if set to 3
        */
        #define GOOGLE_STRIP_LOG 0

        #define LOGD(format, ...) do{ \
                                        char buffer[1024] = {0};  \
                                        snprintf(buffer, 1024, "" format, ##__VA_ARGS__); \
                                        DLOG(INFO) << buffer; \
                                    }while(false);

        #define LOGI(format, ...) do{ \
                                        char buffer[1024] = {0};  \
                                        snprintf(buffer, 1024, "" format, ##__VA_ARGS__); \
                                        LOG(INFO) << buffer; \
                                    }while(false);

        #define LOGW(format, ...) do{ \
                                        char buffer[1024] = {0};  \
                                        snprintf(buffer, 1024, "" format, ##__VA_ARGS__); \
                                        LOG(WARNING) << buffer; \
                                    }while(false);

        #define LOGE(format, ...) do{ \
                                        char buffer[1024] = {0};  \
                                        snprintf(buffer, 1024, "" format, ##__VA_ARGS__); \
                                        LOG(ERROR) << buffer; \
                                    }while(false);
    #endif //__cplusplus

    /*
    * use CLOG for c-codes
    */
    #ifdef NDEBUG
        #define CLOG(format, ...) 
    #else  // !NDEBUG
        #define CLOG(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
    #endif

#elif GLOG_0_ALOG_1 == 1
    #include <android/log.h>

    #define LOGD(format, ...) __android_log_print(ANDROID_LOG_DEBUG, TAG,\
        "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
    #define LOGI(format, ...) __android_log_print(ANDROID_LOG_INFO, TAG,\
            "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
    #define LOGW(format, ...) __android_log_print(ANDROID_LOG_WARN, TAG,\
            "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);
    #define LOGE(format, ...) __android_log_print(ANDROID_LOG_ERROR, TAG,\
            "[%s: %s: %d]: " format, __FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);

#endif

#endif