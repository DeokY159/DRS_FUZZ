#!/usr/bin/env bash
set -e

# helper to build with ASan flags
build_asan() {
  local proj_dir=$1
  cd "${proj_dir}"
  mkdir -p build && cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER=clang-16 \
    -DCMAKE_CXX_COMPILER=clang++-16 \
    -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1" \
    -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1"
  make -j"$(nproc)" && make install
}

### [FastDDS] Install
FASTDDS_WS=/fastdds_ws
SRC_FAST=${FASTDDS_WS}/src
mkdir -p "${SRC_FAST}"
cd "${SRC_FAST}"
### [FastDDS] Build
git clone https://github.com/eProsima/foonathan_memory_vendor.git
git clone https://github.com/eProsima/Fast-CDR.git
git clone https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS && git submodule update --init --recursive
build_asan "${SRC_FAST}/foonathan_memory_vendor"
build_asan "${SRC_FAST}/Fast-CDR"
build_asan "${SRC_FAST}/Fast-DDS"

### [CycloneDDS] Install
CY_WS=/cyclonedds_ws
SRC_CYC="${CY_WS}/src"
mkdir -p "${SRC_CYC}"
cd "${SRC_CYC}"
### [CycloneDDS] Build
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
build_asan "${SRC_CYC}/cyclonedds"

### ASAN Option
export ASAN_OPTIONS="detect_leaks=0:symbolize=1:abort_on_error=0:handle_segv=0:check_initialization_order=0:verify_asan_link_order=0:detect_container_overflow=1"