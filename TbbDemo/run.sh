#!/bin/sh
reset

DEBUG_BIN_PATH=build/Debug/x86_64-linux/TbbDemo
RELEASE_BIN_PATH=build/Release/x86_64-linux/TbbDemo

# add environment variable
export LD_LIBRARY_PATH="${DEBUG_BIN_PATH}/tbb_cmake_build/tbb_cmake_build_debug;${RELEASE_BIN_PATH}/tbb_cmake_build/tbb_cmake_build_release;${LD_LIBRARY_PATH}"
echo "tbb library path: ${LD_LIBRARY_PATH}"

# run binary file
${DEBUG_BIN_PATH}/tbb_test