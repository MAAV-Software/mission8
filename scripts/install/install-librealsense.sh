#!/bin/bash

# Get location of gnc
GNC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
GNC_DIR="$(dirname "$GNC_DIR")"

sudo apt install libusb-1.0-0-dev pkg-config -y
sudo apt-get install libglfw3-dev -y

cd /tmp/
git clone -b 'v2.19.2' https://github.com/IntelRealSense/librealsense.git
cd librealsense/

# For people who have built-in cameras on their device
# These three lines must be run first for the kernel patches to
# go through
sudo modprobe -r uvcvideo
sudo modprobe -r videobuf2_core
sudo modprobe -r videodev

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo apt install libssl-dev -y
./scripts/patch-realsense-ubuntu-lts.sh

mkdir build && cd build
cmake ../

make
sudo make install

echo "YEE. Have a nice day! =)"

cd $GNC_DIR
