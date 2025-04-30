#!/usr/bin/env bash
set -e

# 1) foonathan_memory_vendor install
cd /fastdds_ws/src
git clone https://github.com/eProsima/foonathan_memory_vendor.git
mkdir -p foonathan_memory_vendor/build
cd foonathan_memory_vendor/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=clang-16 \
  -DCMAKE_CXX_COMPILER=clang++-16
make -j$(nproc) && make install

# 2) Fast-CDR + Fast-DDS clone
cd /fastdds_ws/src
git clone https://github.com/eProsima/Fast-CDR.git
git clone https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS && git submodule update --init --recursive

# 3) Fast-CDR build
cd /fastdds_ws/src/Fast-CDR
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=clang-16 \
  -DCMAKE_CXX_COMPILER=clang++-16 \
  -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1"
make -j$(nproc) && make install

# 4) Fast-DDS build
cd /fastdds_ws/src/Fast-DDS
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=clang-16 \
  -DCMAKE_CXX_COMPILER=clang++-16 \
  -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1"
make -j$(nproc) && make install