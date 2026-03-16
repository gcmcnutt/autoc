#! /bin/bash
set -ex

# Clean debug rebuild — autoc + crrcsim
cd "$(dirname "$0")"
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_CRRCSIM=ON ..
make
