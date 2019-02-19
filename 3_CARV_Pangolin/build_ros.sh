#!/usr/bin/env bash
echo "Building ROS nodes"

cd Examples/ROS/CARV_Pangolin
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Debug
make clean
make -j 2
