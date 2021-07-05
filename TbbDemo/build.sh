#!/bin/sh
reset
PROJECT_NAME=TbbDemo
BUILD_TYPE=Debug
BUILD_PLATFORM=x86_64-linux
BUILD_CMD=build
#TARGET_OS support linux and android
TARGET_OS=linux
#TARGET_ARCH support x86_64 and armv8
TARGET_ARCH=x86_64
#ANDROID_NDK_DIR=/home/shaquille/android-ndk-r16b
# ANDROID_NDK_DIR=/Users/wyb/Library/Android/sdk/ndk/16.1.4479499

while getopts ":c:t:a:o:q" opt
do
    case $opt in
        c)
        BUILD_CMD=$OPTARG
        ;;    
        t)
        BUILD_TYPE=$OPTARG
        ;;
        a)
        TARGET_ARCH=$OPTARG
        ;;
        o)
        TARGET_OS=$OPTARG
        ;;        
        ?)
        echo "unknow parameter: $opt"
        exit 1;;
    esac
done

CORE_COUNT=$(cat /proc/cpuinfo | grep processor | wc -l)
echo "CORE_COUNT ${CORE_COUNT}"

CUR_DIR_PATH=${PWD}
ORION_TBB_ROOT=${CUR_DIR_PATH}/orion_tbb
TBB_CMAKE_BUILD_DIR=tbb_cmake_build
BUILD_PLATFORM=${TARGET_ARCH}-${TARGET_OS}


mkdir -p ./build/${BUILD_TYPE}/${BUILD_PLATFORM}/${PROJECT_NAME}
cd ./build/${BUILD_TYPE}/${BUILD_PLATFORM}/${PROJECT_NAME}
echo entring ${PWD} for ${BUILD_CMD} ${PROJECT_NAME} start

BUILD_CMD_LINE="-DCMAKE_INSTALL_PREFIX=${PWD}/install -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DPROJECT_BUILD_ROOT=${PWD} -DTBB_CMAKE_BUILD_DIR=${TBB_CMAKE_BUILD_DIR} -DTARGET_ARCH=${TARGET_ARCH} -DTARGET_OS=${TARGET_OS} ../../../../"
if [ "$TARGET_OS" == "android" ] ; then
    if [ "$NDK_ROOT" == "" ] ; then
        export NDK_ROOT=${ANDROID_NDK_DIR}
    else
        ANDROID_NDK_DIR=${NDK_ROOT}
    fi
    export TRIPLE=aarch64-linux-android
    if [ "$TARGET_ARCH" == "armv8" ] ; then
        export TRIPLE=aarch64-linux-android
        ANDROID_ABI_FORMAT="arm64-v8a"
        ANDROID_API_VERSION=27
        #ANDROID_OPENCV_DIR=${ANDROID_OPENCV_DIR_PREFIX}/abi-arm64-v8a
    else
        ANDROID_ABI_FORMAT="armeabi-v7a"
        ANDROID_API_VERSION=22
        #ANDROID_OPENCV_DIR=${ANDROID_OPENCV_DIR_PREFIX}/abi-armeabi-v7a
    fi
    BUILD_CMD_LINE="-DCMAKE_TOOLCHAIN_FILE=${ANDROID_NDK_DIR}/build/cmake/android.toolchain.cmake -DANDROID_ABI=${ANDROID_ABI_FORMAT} -DANDROID_PLATFORM=android-${ANDROID_API_VERSION} -DANDROID_API_VERSION=${ANDROID_API_VERSION} -DANDROID_STL=c++_shared -DANDROID_NDK_DIR=${ANDROID_NDK_DIR} -DOpenCV_DIR=${ANDROID_OPENCV_DIR} ${BUILD_CMD_LINE}"
elif [ "$TARGET_ARCH" == "armv7" ] || [ "$TARGET_ARCH" == "armv8" ] ; then
    echo "x86_64-linux platform"
elif [ "$TARGET_ARCH" == "x86_64" ] && [ "$TARGET_OS" == "linux" ] ; then
    echo "x86_64-linux platform"
else
    echo "unknown platform"
fi

if [ "$BUILD_CMD" == "build" ]; then
    cmake ${BUILD_CMD_LINE}
    make -j8
    # make install
elif 
    [ "$BUILD_CMD" == "clean" ]; then
    make clean
    rm -rf ./tests/CMakeFiles
    rm -rf ./tests/cmake_install.cmake
    rm -rf ./tests/Makefile
    rm -rf ./tools/CMakeFiles
    rm -rf ./tools/cmake_install.cmake
    rm -rf ./tools/Makefile 
    rm -rf ./examples/CMakeFiles
    rm -rf ./examples/cmake_install.cmake
    rm -rf ./examples/Makefile
    rm -rf ./demos/CMakeFiles
    rm -rf ./demos/cmake_install.cmake
    rm -rf ./demos/Makefile
    rm -rf ./CMakeFiles
    rm -rf ./CMakeCache.txt
    rm -rf ./cmake_install.cmake
    rm -rf ./CTestTestfile.cmake
    rm -rf ./tests/CTestTestfile.cmake
    rm -rf ./install_manifest.txt
    rm -rf ./install/*
    rm -rf ./${TBB_CMAKE_BUILD_DIR}
    rm -rf ./Makefile
elif 
    [ "$BUILD_CMD" == "test" ]; then
    make test
else
    echo "unknown cmd"
fi

cd ${CUR_DIR_PATH}
echo leaving ${PWD} for ${BUILD_CMD} ${PROJECT_NAME} end