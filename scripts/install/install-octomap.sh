#!/bin/bash
#
# Install apriltag to third party

# Get location of software
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$SCRIPT_DIR/../.."
OCTOMAP_DIR="${SOFTWARE_DIR}/thirdparty/octomap"

sudo apt-get install libqglviewer-dev -y

cd /tmp
git clone https://github.com/OctoMap/octomap.git
cd octomap
git checkout v1.9.0
mkdir -p build
cd build
cmake ..    \
    -DCMAKE_BUILD_TYPE=Release  \
    -DOCTOVIS_QT5=ON    \
    -DOCTOMAP_OMP=ON    \
    -DCMAKE_INSTALL_PREFIX=${OCTOMAP_DIR}

# Install
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))
sudo make install
