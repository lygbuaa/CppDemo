#!/bin/sh
reset

BIN_PATH=./build/Debug/x86_64-linux/TbbDemo

# add environment variable
export LD_LIBRARY_PATH=${BIN_PATH}/tbb_cmake_build/tbb_cmake_build_debug
echo "tbb library path: ${LD_LIBRARY_PATH}"

# run binary file
${BIN_PATH}/tbb_test