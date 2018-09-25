#!/bin/bash

# MAAV Software setup script
# This script will set up Software for you. You only need to run it once
# This script assumes it is in software/scripts
# DO NOT MOVE THIS SCRIPT
# Should work for Ubuntu and Arch-like systems
# Authors: Martin Deegan (mdeegan),
#	   Romario Pashollari (rpash)

# Get location of software
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"

# Add MAAV software as upstream
echo "Setting upstream to maav/software..."
git remote add upstream git@gitlab.eecs.umich.edu:maav/software.git

# Get submodules
echo "Initializing submodules..."
git submodule update --init --recursive

# Install dependencies
sudo apt install cmake \
                 cmake-curses-gui \
                 clang-format \
                 libyaml-cpp-dev \
                 curl \
                 libcurl4-openssl-dev \
                 ffmpeg \
                 libglew-dev \
                 libboost-test-dev \
                 libboost-program-options-dev \
                 libev-dev \
                 doxygen \
                 libgtkmm-3.0-dev \
                 libudev-dev \
                 libglm-dev \
                 libusb-1.0-0-dev \
                 libusb-1.0-doc \
                 libusb-1.0-0-dbg \
                 python3 \
                 python3-pip \
                 sl -y # system libraries

# Install custom deps
${SOFTWARE_DIR}/scripts/install/install-cmake.sh
${SOFTWARE_DIR}/scripts/install/install-g++7.sh
${SOFTWARE_DIR}/scripts/install/install-zcm.sh
${SOFTWARE_DIR}/scripts/install/install-librealsense.sh
${SOFTWARE_DIR}/scripts/install/install-pangolin.sh
${SOFTWARE_DIR}/scripts/install/install-pcl.sh
${SOFTWARE_DIR}/scripts/install/install-opencv3.sh
${SOFTWARE_DIR}/scripts/install/install-lcm.sh
${SOFTWARE_DIR}/scripts/install/install-librealsense-legacy.sh
