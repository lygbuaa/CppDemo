//
// Created by llm on 20-7-8.
//
#include <jni.h>
#include "utils/glogging.h"
#include <assert.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include "V4L2/V4L2Camera.h"
#include "PedestrianTrackerNode.h"
#include "GlobalConstants.h"

//定义日志打印宏函数
#define LOG_TAG "FFMediaPlayer"
//#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
//#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
//#define LOGD(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

//#define VIDEO_FILE "/dev/video3"
//#define IMAGEWIDTH      640
//#define IMAGEHEIGHT     480
//#define PIX_FORMATE     V4L2_PIX_FMT_YUYV

V4L2Camera* v4l2Camera = nullptr;
PedestrianTracker::PedestrianTrackerNode* pTracker_ = nullptr;
JavaVM *javaVM = nullptr;

static void native_init(JNIEnv *env,jobject thiz) {

    v4l2Camera = new V4L2Camera();
    pTracker_ = new PedestrianTracker::PedestrianTrackerNode();
    //由v4l2负责释放
    JavaCallHelper* javaCallHelper = new JavaCallHelper(javaVM, env, thiz);
    v4l2Camera->setListener(javaCallHelper);
}

static void native_release(JNIEnv *env) {

    if (v4l2Camera != nullptr) {
        v4l2Camera->setListener(0);
        delete v4l2Camera;
        v4l2Camera = nullptr;
    }

    if(pTracker_ != nullptr){
        delete pTracker_;
        pTracker_ = nullptr;
    }
}

static jint native_open(JNIEnv *env, jobject thiz) {
    using namespace PedestrianTracker;
    int ret = ERROR_OPEN_FAIL;
    if (v4l2Camera != 0) {
        ret = v4l2Camera->Open(GlobalConstants::VIDEO_FILE_, GlobalConstants::IMAGE_WIDTH_, GlobalConstants::IMAGE_HEIGHT_, PIX_FORMATE_);
    }

    return ret;
}

static void native_close(JNIEnv *env, jobject thiz) {
    if (v4l2Camera != 0) {
        v4l2Camera->Close();
    }
}

static jobject native_getParameters(JNIEnv *env, jobject thiz) {

    if (v4l2Camera == 0) {
        return 0;
    }
    std::list<Parameter> parameters = v4l2Camera->getParameters();

    jclass list_class = env->FindClass("java/util/ArrayList");
    if (list_class == NULL) {
        return 0;
    }

    jmethodID list_costruct = env->GetMethodID(list_class , "<init>","()V"); //获得得构造函数Id
    jmethodID list_add = env->GetMethodID(list_class, "add", "(Ljava/lang/Object;)Z");
    jobject list_obj = env->NewObject(list_class , list_costruct); //创建一个Arraylist集合对象

    jclass parameter_cls = env->FindClass("pri/tool/bean/Parameter");//获得类引用
    //获得该类型的构造函数  函数名为 <init> 返回类型必须为 void 即 V
    jmethodID parameter_costruct = env->GetMethodID(parameter_cls , "<init>", "(ILjava/util/ArrayList;)V");

    jclass frame_cls = env->FindClass("pri/tool/bean/Frame");//获得类引用
    //获得该类型的构造函数  函数名为 <init> 返回类型必须为 void 即 V
    jmethodID frame_costruct = env->GetMethodID(frame_cls , "<init>", "(IILjava/util/ArrayList;)V");

    jclass frameRate_cls = env->FindClass("pri/tool/bean/FrameRate");//获得类引用
    //获得该类型的构造函数  函数名为 <init> 返回类型必须为 void 即 V
    jmethodID frameRate_costruct = env->GetMethodID(frameRate_cls , "<init>", "(II)V");

    for (Parameter parameter : parameters) {

        jobject listFrame_obj = env->NewObject(list_class , list_costruct); //创建一个Arraylist集合对象
        for (Frame frame : parameter.frames) {

            jobject listFrameRate_obj = env->NewObject(list_class , list_costruct); //创建一个Arraylist集合对象
            for (FrameRate frameRate : frame.frameRate) {
                jobject frameRate_obj = env->NewObject(frameRate_cls, frameRate_costruct, frameRate.numerator, frameRate.denominator);
                env->CallBooleanMethod(listFrameRate_obj , list_add , frameRate_obj);
            }

            jobject frame_obj = env->NewObject(frame_cls, frame_costruct, frame.width, frame.height, listFrameRate_obj);
            env->CallBooleanMethod(listFrame_obj , list_add , frame_obj);
        }

        //TODO:完善类型映射
        int format;
        switch(parameter.pixFormat) {
            case V4L2_PIX_FMT_YUYV:
                format = YUYV;
                break;
            default:
                format = parameter.pixFormat;
                break;
        }
        jobject parameter_obj = env->NewObject(parameter_cls, parameter_costruct, format, listFrame_obj);
        env->CallBooleanMethod(list_obj, list_add, parameter_obj);
    }

    return list_obj;
}

static jint native_setPreviewSize(JNIEnv *env, jobject thiz, jint width, jint height, jint pixformat) {
    if (v4l2Camera == 0) {
        return ERROR_CAPABILITY_UNSUPPORT;
    }

    return v4l2Camera->setPreviewSize(width, height, V4L2_PIX_FMT_YUYV);
}

static int native_setSurface(JNIEnv *env, jobject thiz, jobject surface) {
    if (v4l2Camera == 0) {
        return ERROR_CAPABILITY_UNSUPPORT;
    }

    ANativeWindow *window = ANativeWindow_fromSurface(env, surface);

    v4l2Camera->setSurface(window);

    return 0;
}

static int native_startPreview(JNIEnv *env, jobject thiz) {
    if (v4l2Camera == 0) {
        return ERROR_CAPABILITY_UNSUPPORT;
    }
    int ret = 0;
    ret = v4l2Camera->Init();
    if (ret != 0 ) {
        LOGE("startPreview init fail");
    }

    /* do the job in PedestrianTrackerNode */
    v4l2Camera->StartStreaming();
    pTracker_ -> SetCameraPtr(v4l2Camera);
    pTracker_ -> Start();

    return 0;
}

static int native_stopPreview(JNIEnv *env, jobject thiz) {
    if (v4l2Camera == 0) {
        return ERROR_CAPABILITY_UNSUPPORT;
    }

    v4l2Camera->StopStreaming();
    pTracker_ -> Stop();

    return 0;
}

static int native_capture(JNIEnv *env, jobject thiz, jstring path) {
    if(pTracker_){
        const char* cpath = env->GetStringUTFChars(path, nullptr);
        return pTracker_ -> CaptureMono(cpath);
    }
    return -1;
}

static void native_setTrack(JNIEnv *env, jobject thiz, jboolean enable) {
    if(pTracker_){
        pTracker_ -> SetTrack(enable);
    }
}

static void native_setRecord(JNIEnv *env, jobject thiz, jboolean enable) {
    if(pTracker_){
        pTracker_ -> SetRecord(enable);
    }
}

static void native_setPlayback(JNIEnv *env, jobject thiz, jboolean enable) {
    if(pTracker_){
        pTracker_ -> SetPlayback(enable);
    }
}

static JNINativeMethod gMethods[] = {
{"native_init",         "()V",                              (void *)native_init},
{"native_release",         "()V",                              (void *)native_release},
{"native_open",         "()I",                              (void *)native_open},
{"native_close",         "()V",                              (void *)native_close},
{"native_getParameters",         "()Ljava/util/ArrayList;",                              (void *)native_getParameters},
{"native_setPreviewSize",         "(III)I",                              (void *)native_setPreviewSize},
{"native_setSurface",         "(Ljava/lang/Object;)I",                              (void *)native_setSurface},
{"native_startPreview",         "()I",                              (void *)native_startPreview},
{"native_stopPreview",         "()I",                              (void *)native_stopPreview},
{"native_capture",         "(Ljava/lang/String;)I",                              (void *)native_capture},
{"native_setTrack",         "(Z)V",                              (void *)native_setTrack},
{"native_setRecord",         "(Z)V",                              (void *)native_setRecord},
{"native_setPlayback",         "(Z)V",                              (void *)native_setPlayback},
};

//Ljava/lang/Object;
//Ljava/util/ArrayList;
static int registerNativeMethods(JNIEnv* env, const char* className, JNINativeMethod* gMethods, int methodsNum) {
    jclass clazz;
    //找到声明native方法的类
    clazz = env->FindClass(className);
    if(clazz == NULL){
        return JNI_FALSE;
    }
    //注册函数 参数：java类 所要注册的函数数组 注册函数的个数
    if(env->RegisterNatives(clazz, gMethods, methodsNum) < 0){
        return JNI_FALSE;
    }
    return JNI_TRUE;
}

static int register_native_methods(JNIEnv* env){
    //指定类的路径，通过FindClass 方法来找到对应的类
    const char* className  = "pri/tool/v4l2camera/V4L2Camera";

    return registerNativeMethods(env, className, gMethods, sizeof(gMethods)/ sizeof(gMethods[0]));
}

jint JNI_OnLoad(JavaVM* vm, void* /* reserved */)
{
    JNIEnv* env = NULL;
    jint result = -1;

    javaVM = vm;

    if (vm->GetEnv((void**) &env, JNI_VERSION_1_4) != JNI_OK) {
        LOGE("ERROR: GetEnv failed\n");
        goto bail;
    }
    assert(env != NULL);

    if (register_native_methods(env) < 0) {
        LOGE("ERROR: register_native_methods failed");
        goto bail;
    }
    /* success -- return valid version number */
    result = JNI_VERSION_1_4;

    bail:
    return result;
}
