cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON)

project(TbbDemo C CXX)
ADD_DEFINITIONS(-std=c++11)

message(STATUS "current cmake build type: ${CMAKE_BUILD_TYPE}")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fPIC")
else(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O0 -fPIC")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")

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

# dependencies
set(COMMON_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../common")
set(GTEST_DIR "${COMMON_ROOT_DIR}/gtest")
file(GLOB_RECURSE GTEST_SRC "${GTEST_DIR}/gtest-all.cpp")

set(TBB_ROOT ${COMMON_ROOT_DIR}/oneTBB)
message(STATUS "Current CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
if(CMAKE_SYSTEM_NAME STREQUAL "Android")
     set(TBB_BUILD_COMPILER "clang")
else()
     set(TBB_BUILD_COMPILER "gcc")
endif()

include(${TBB_ROOT}/cmake/TBBBuild.cmake)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DDEBUG -O0")
     set(TBB_FIND_RELEASE_ONLY 0)
     set(TBB_GENERATE_CFG "_debug")
     set(TBB_GENERATE_SUFFIX "_debug")
     set(TBB_MAKE_ARGS tbb_build_dir=${PROJECT_BUILD_ROOT}/${TBB_CMAKE_BUILD_DIR} tbb_build_prefix=${TBB_CMAKE_BUILD_DIR} TBB_DEBUG_DIR=${PROJECT_BUILD_ROOT}/${TBB_CMAKE_BUILD_DIR}/${TBB_CMAKE_BUILD_DIR}_debug cfg=debug)
else()
     set(TBB_GENERATE_CFG "_release")
     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DNDEBUG")
     set(TBB_FIND_RELEASE_ONLY 1)
     set(TBB_MAKE_ARGS tbb_build_dir=${PROJECT_BUILD_ROOT}/${TBB_CMAKE_BUILD_DIR} tbb_build_prefix=${TBB_CMAKE_BUILD_DIR} TBB_RELEASE_DIR=${PROJECT_BUILD_ROOT}/${TBB_CMAKE_BUILD_DIR}/${TBB_CMAKE_BUILD_DIR}_release cfg=release)
endif()

if(TARGET_OS STREQUAL "android")
     set(TBB_MAKE_ARGS ${TBB_MAKE_ARGS} arch=${ANDROID_ABI} compiler=${TBB_BUILD_COMPILER} api_version=${ANDROID_API_VERSION} target=android)
endif()

tbb_build(TBB_ROOT ${TBB_ROOT} CONFIG_DIR TBB_DIR MAKE_ARGS ${TBB_MAKE_ARGS})
find_package(TBB REQUIRED)

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
