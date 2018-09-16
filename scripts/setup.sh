#!/bin/bash

# MAAV Software setup script
# This script will set up Software for you. You only need to run it once
# This script assumes it is in gnc/bash
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
                 libopencv-dev \
                 curl \
                 libglew-dev \
                 libboost-test-dev -y

# Install custom deps
${SOFTWARE_DIR}/scripts/install/install-g++7.sh
${SOFTWARE_DIR}/scripts/install/install-pangolin.sh
${SOFTWARE_DIR}/scripts/install/install-librealsense.sh
${SOFTWARE_DIR}/scripts/install/install-zcm.sh

