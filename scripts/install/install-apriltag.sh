#!/bin/bash
#
# Install apriltag to third party

# Get location of software
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"

INCLUDE_DIR=$SOFTWARE_DIR/thirdparty/apriltags/include
LIB_DIR=$SOFTWARE_DIR/thirdparty/apriltags/lib

cd /tmp
git clone https://github.com/AprilRobotics/apriltags.git apriltags
cd apriltags

# Checkout a known compatible and stable commit
git checkout fa3b50f3f1e4acaff0dd6e501ce9b5ca251dc3fd

# Use sed to modify the makefile to not build examples
sed -i 's/all: apriltag_demo opencv_demo/all: /' example/Makefile


# Install
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))

# Custom installation
mkdir -p $INCLUDE_DIR/common
mkdir -p $LIB_DIR
for i in *.h; do
	cp $i $INCLUDE_DIR
	echo "$INCLUDE_DIR/$i"
done
for i in common/*.h; do
	cp $i $INCLUDE_DIR/common
	echo "$INCLUDE_DIR/common/$i"
done
for i in *.so; do
	cp $i $LIB_DIR
	echo "$LIB_DIR/$i"
done
