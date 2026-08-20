# WSL2 x86_64 -> Pi aarch64 cross toolchain (research.md R7, quickstart.md §3).
#
# The point of this file is how LITTLE it has to do. `core/`, `tools/` and `tests/` are zero-dependency
# C11, so they cross with nothing but the distro cross-gcc and no sysroot at all. Only `io/` + `app/`
# need a sysroot, because that is where libcamera lives — which is the dependency boundary earning its
# keep (plan.md §Structure Decision).
#
#   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#   # core/tools/tests only — no sysroot needed:
#   cmake -S . -B build-cross -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF \
#         -DCMAKE_TOOLCHAIN_FILE=firmware/beacon-receiver/cmake/aarch64-linux-gnu.cmake
#   # adding app/ (libcamera) needs a sysroot rsync'd from the Pi:
#   rsync -a pi@<pi>:/usr/include pi@<pi>:/usr/lib ~/pi-sysroot/
#   cmake ... -DCMAKE_SYSROOT=$HOME/pi-sysroot

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Search host paths for programs, target paths for everything else. Without this the cross build happily
# finds x86_64 libraries and fails at link with an error that reads like a missing package.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BEFORE)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
