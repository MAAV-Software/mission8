#!/bin/bash

# MAAV GNC setup script
# This script will set up GNC for you. You only need to run it once
# This script assumes it is in gnc/bash
# DO NOT MOVE THIS SCRIPT
# Should work for Ubuntu and Arch-like systems
# Authors: Martin Deegan (mdeegan),
#	   Romario Pashollari (rpash)

# Get location of gnc
GNC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
GNC_DIR="$(dirname "$GNC_DIR")"

# Add MAAV gnc as upstream
echo "Setting upstream to maav/gnc..."
git remote add upstream git@gitlab.eecs.umich.edu:maav/gnc.git

# Get submodules
echo "Getting submodules..."
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
${GNC_DIR}/bash/install-g++7.sh
${GNC_DIR}/bash/install-eigen3.3.5.sh
${GNC_DIR}/bash/install-pangolin.sh
${GNC_DIR}/bash/install-librealsense.sh

# Get TUMS dataset
if ! [ -f $GNC_DIR/datasets/TUMS/rgbd_dataset_freiburg1_xyz.tgz ]; then
    echo "Downloading TUMS dataset..."
    mkdir -p $GNC_DIR/datasets/TUMS
    curl -o $GNC_DIR/datasets/TUMS/rgbd_dataset_freiburg1_xyz.tgz \
    http://filecremers3.informatik.tu-muenchen.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
fi
echo "Extracting TUMS dataset..."
tar -xzf $GNC_DIR/datasets/TUMS/rgbd_dataset_freiburg1_xyz.tgz -C $GNC_DIR/datasets/TUMS

# # Setting up ORB Vocabulary
echo "Setting up ORB Vocabulary..."
if ! [ -f $GNC_DIR/Vocabulary/ORBvoc.txt.tar.gz ]; then
    echo "Downloading ORB vocabulary..."
    mkdir -p $GNC_DIR/Vocabulary
    curl -o $GNC_DIR/Vocabulary/ORBvoc.txt.tar.gz \
    "https://doc-0k-5o-docs.googleusercontent.com/docs/securesc/ha0ro937gcuc7l7deffksulhg5h7mbp1/ls6uivmvh8fhobg79cokn2a5urb32935/1535335200000/05968329248168867851/*/1bh87Ug7bdNskxalAXprRcCSYc2YNnp2_"
fi

echo "Extracting ORB vocabulatry..."
tar -xzf $GNC_DIR/Vocabulary/ORBvoc.txt.tar.gz -C $GNC_DIR/Vocabulary
