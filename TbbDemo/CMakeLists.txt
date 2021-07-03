cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON)

project(TbbDemo C CXX)

message(STATUS "current cmake build type: ${CMAKE_BUILD_TYPE}")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message(STATUS "TARGET_OS ${TARGET_OS}, TARGET_ARCH ${TARGET_ARCH}")
if(TARGET_ARCH STREQUAL "x86_64")
     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=x86-64")
elseif(TARGET_OS STREQUAL "android")
     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DANDROID -fexceptions -frtti  -Wno-unused-private-field")
     if(TARGET_ARCH STREQUAL "armv8")
          set(ANDROID_ABI  "arm64-v8a")
     else()
          set(ANDROID_ABI  "armeabi-v7a")
     endif()
     message("ANDROID_ABI: ${ANDROID_ABI}")
else()
     message(FATAL_ERROR "unsupported TARGET_OS ${TARGET_OS}, TARGET_ARCH ${TARGET_ARCH}")
endif()

message(STATUS "Current CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
if(CMAKE_SYSTEM_NAME STREQUAL "Android")
     set(TBB_BUILD_COMPILER "clang")
else()
     set(TBB_BUILD_COMPILER "gcc")
endif()

# dependencies
set(COMMON_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../common")
set(GTEST_DIR "${COMMON_ROOT_DIR}/gtest")
file(GLOB_RECURSE GTEST_SRC "${GTEST_DIR}/gtest-all.cpp")

# source files
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/test_tbb.cc")


include_directories(
    ${COMMON_ROOT_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_executable(
    tbb_test
    ${SRC_FILES}
    ${GTEST_SRC}
)

target_link_libraries(
    tbb_test
    pthread
    glog
)
