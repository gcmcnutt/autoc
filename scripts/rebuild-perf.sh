#! /bin/bash
set -ex

# Clean optimized rebuild — autoc + crrcsim
cd "$(dirname "$0")/.."
rm -rf build
mkdir build
cd build
cmake -DPERFORMANCE_BUILD=ON ..
make
