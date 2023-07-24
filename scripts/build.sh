#!/usr/bin/env bash

MYPWD=$(pwd)
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`

set -x
set -e

mkdir -p thirdparty

cd $MYPWD/thirdparty
# https://stackoverflow.com/a/36499031
url="https://github.com/libLAS/libLAS.git"
folder="libLAS"
if ! git clone "${url}" "${folder}" 2>/dev/null && [ -d "${folder}" ] ; then
    rm -rf "${folder}"
    git clone "${url}" "${folder}"
fi
cd libLAS && mkdir -p build && cd build && cmake  .. && make -j$NUM_CORES && make install

cd $MYPWD/thirdparty
# https://stackoverflow.com/a/36499031
url="https://github.com/wjakob/tbb.git"
folder="tbb"
if ! git clone "${url}" "${folder}" 2>/dev/null && [ -d "${folder}" ] ; then
    rm -rf "${folder}"
    git clone "${url}" "${folder}"
fi
cd tbb && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$NUM_CORES && make install

cd $MYPWD/thirdparty
# https://stackoverflow.com/a/36499031
url="https://github.com/oneapi-src/oneTBB.git"
folder="oneTBB"
if ! git clone "${url}" "${folder}" 2>/dev/null && [ -d "${folder}" ] ; then
    rm -rf "${folder}"
    git clone "${url}" "${folder}"
fi
cd oneTBB && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$NUM_CORES && make install

cd $MYPWD/thirdparty
url="https://github.com/seladb/PcapPlusPlus/archive/v22.11.tar.gz"
file="v22.11.tar.gz"
if ! wget "${url}" 2>/dev/null && [ -d "${file}" ] ; then
    rm -rf "${file}"
    wget "${url}"
fi
tar -xvf v22.11.tar.gz
cd PcapPlusPlus-22.11 && yes no | ./configure-linux.sh && make -j$NUM_CORES && make install
