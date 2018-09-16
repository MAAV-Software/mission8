#!/bin/bash
cd /tmp

sudo apt install -y libglfw3* \
		    libflann1.8 \
		    libflann-dev \
		    libboost-all-dev \
		    libvtk6-dev \
		    libproj-dev \
		    libqhull-dev

# Install librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout legacy
mkdir build
cd build
cmake ..
make -j
sudo make install
cd /tmp
rm -rf librealsense
